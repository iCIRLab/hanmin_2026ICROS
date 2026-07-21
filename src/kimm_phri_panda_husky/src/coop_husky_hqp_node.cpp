// 차동구동 모바일 매니퓰레이터 협업 운반용 HQP adapter.
// 통합 MuJoCo 상태를 로봇별 상태로 나누고 Whole-Body HQP 명령을 합친다.

#include <pinocchio/fwd.hpp>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "mujoco_ros_msgs/JointSet.h"
#include "mujoco_ros_msgs/SensorState.h"

#include "kimm_phri_panda_husky/panda_husky_hqp.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <set>

using namespace std;
using namespace Eigen;

namespace {

// FrankaHuskyWrapper가 기대하는 단일 로봇 상태 블록.
constexpr int kPosBlock = 20;  // 7 odom (x,y,z,qw,qx,qy,qz) + 4 wheel + 7 arm + 2 finger
constexpr int kVelBlock = 19;  // 6 odom + 4 wheel + 7 arm + 2 finger

RobotController::FrankaHuskyWrapper* ctrl_ = nullptr;

double time_ = 0.0;
int qpos_offset_ = 0;
int qvel_offset_ = 0;
bool state_received_ = false;
bool controller_initialized_ = false;
bool pending_mode_valid_ = false;
int pending_mode_ = 0;
double base_x_ = 0.0;
// The TSID model uses a planar base at world z=0, while MuJoCo publishes the
// settled floating-base height (about -0.118 m in the cooperative scenes).
// Apply that measured world-z translation whenever a controller-frame pose is
// compared with or published for MuJoCo geometry.
double base_world_z_ = 0.0;
double arm_q_[7] = {0,0,0,0,0,0,0};   // posture reference 확인용 관절값

ros::Publisher command_pub_;
ros::Publisher ee_grasp_pose_pub_;
ros::Publisher link7_pose_pub_;
ros::Publisher elbow_pose_pub_;
mujoco_ros_msgs::JointSet command_msg_;

// 선택 사항: grasp interaction wrench 기반 하중 보상.
// sensor는 실제 gripper/FT sensor 경로, coupling은 weld simulation 근사 경로다.
bool fext_enabled_ = false;
std::string fext_source_ = "sensor";   // "coupling" | "sensor"
bool fext_sample_valid_ = false;
ros::WallTime last_fext_update_;
double fext_timeout_ = 0.25;

// object grasp와 EE grasp 차이를 spring force로 근사한다.
int      object_qpos_offset_ = 40;   // object freejoint start in combined joint_states
Vector3d couple_grasp_offset_(0.0, -0.85, 0.0);  // object-local grasp frame (follower: -y)
double   couple_stiffness_ = 0.0;    // N/m: Fext_lin = k * (object_grasp - ee_grasp)
double   couple_force_limit_ = 0.0;  // per-axis |force| clamp (N); 0 = no clamp

// EE force/torque sensor를 사용할 때의 scale/name.
double ee_force_scale_ = 1.0;        // node-side gain/sign on the linear force
double ee_torque_scale_ = 0.0;       // node-side gain/sign on the torque (default off)
std::string force_sensor_name_;      // e.g. "follower_ee_force"
std::string torque_sensor_name_;     // e.g. "follower_ee_torque"

MatrixXd robot_mass_;
VectorXd robot_nle_;
VectorXd franka_qacc_;
VectorXd husky_qacc_;
VectorXd franka_torque_;
// mode235 하중 보상용 feedforward torque.
int    cur_hqp_mode_ = 0;
bool   fext_comp_enabled_ = false;
double fext_comp_gain_ = 1.0;   // JWorld^T*Fext_ 보상 gain
Vector3d ee_grasp_offset_(0.0, 0.0, 0.1654);
Quaterniond ee_grasp_quat_(0.0, 1.0, 0.0, 0.0);  // MuJoCo XML quat: w x y z.
std::string ee_position_sensor_name_;
std::string ee_quaternion_sensor_name_;
Vector3d mujoco_ee_grasp_p_(Vector3d::Zero());
Quaterniond mujoco_ee_grasp_q_(Quaterniond::Identity());
bool mujoco_ee_grasp_pose_valid_ = false;
int cm_lift_mode_ = 238;
int transport_mode_ = 235;
bool cm_lift_pending_ = false;
bool cm_transport_ready_ = false;

bool readVector3Param(ros::NodeHandle& pnh, const std::string& name, Vector3d& out) {
    std::vector<double> values;
    if (!pnh.getParam(name, values) || values.size() != 3)
        return false;
    out = Vector3d(values[0], values[1], values[2]);
    return true;
}

bool readQuaternionParam(ros::NodeHandle& pnh, const std::string& name, Quaterniond& out) {
    std::vector<double> values;
    if (!pnh.getParam(name, values) || values.size() != 4)
        return false;
    out = Quaterniond(values[0], values[1], values[2], values[3]);
    if (out.norm() < 1e-9)
        out = Quaterniond::Identity();
    out.normalize();
    return true;
}

bool updateFextSample(const Vector6d& wrench) {
    if (!ctrl_ || !wrench.allFinite()) {
        fext_sample_valid_ = false;
        ROS_WARN_THROTTLE(1.0, "[coop_husky_hqp] rejected non-finite Fext sample");
        return false;
    }
    ctrl_->Fext_update(wrench);
    fext_sample_valid_ = true;
    last_fext_update_ = ros::WallTime::now();
    return true;
}

bool fextSampleFresh() {
    if (!fext_sample_valid_ || last_fext_update_.isZero())
        return false;
    return (ros::WallTime::now() - last_fext_update_).toSec() <= fext_timeout_;
}

void simTimeCallback(const std_msgs::Float32ConstPtr& msg) {
    time_ = static_cast<double>(msg->data);
}

// HQP가 직접 푸는 mode만 실행한다. attach/lift 절차 mode는 orchestration node가 처리한다.
const std::set<int> kImplementedModes = {
    1, 200, 235};

void ctrlModeCallback(const std_msgs::Int32ConstPtr& msg) {
    if (!ctrl_) return;
    const int mode = static_cast<int>(msg->data);
    if (mode == cm_lift_mode_) {
        // CM lift 이후 transport-ready가 오기 전에는 주행 mode 진입을 막는다.
        cm_lift_pending_ = true;
        cm_transport_ready_ = false;
        ROS_INFO("[coop_husky_hqp] CM lift mode %d observed; mode %d will wait for CM transport-ready",
                 cm_lift_mode_, transport_mode_);
    }
    if (kImplementedModes.count(mode) == 0) {
        ROS_INFO_THROTTLE(1.0, "[coop_husky_hqp] ignoring unimplemented ctrl_mode %d (kept current)", mode);
        return;
    }
    if (mode == transport_mode_ && cm_lift_pending_ && !cm_transport_ready_) {
        ROS_WARN_THROTTLE(1.0,
                          "[coop_husky_hqp] blocked mode %d: CM lift is pending and /coop/cm_lift_ready is false",
                          transport_mode_);
        return;
    }
    if (!controller_initialized_) {
        pending_mode_ = mode;
        pending_mode_valid_ = true;
        ROS_INFO("[coop_husky_hqp] queued ctrl_mode %d until first valid robot state", mode);
        return;
    }
    ROS_INFO("[coop_husky_hqp] shared ctrl_mode request -> %d", mode);
    ctrl_->ctrl_update(mode);
    cur_hqp_mode_ = mode;
    if (mode == transport_mode_)
        cm_lift_pending_ = false;
}

void cmTransportReadyCallback(const std_msgs::BoolConstPtr& msg) {
    cm_transport_ready_ = msg->data;
}

// 물체-EE 관계에서 계산한 world-frame task reference.
void eeTargetCallback(const geometry_msgs::Transform::ConstPtr& msg) {
    if (!ctrl_) return;
    Eigen::Vector3d t(msg->translation.x, msg->translation.y, msg->translation.z);
    // Object targets are expressed in the physical MuJoCo world. Calibrate
    // the complete translation mismatch (settled base z plus any URDF/MJCF
    // frame discrepancy) from the exact green site sensor at the current q.
    pinocchio::SE3 link7_pose;
    ctrl_->position(link7_pose);
    const Vector3d predicted_green =
        link7_pose.translation() + link7_pose.rotation() * ee_grasp_offset_;
    if (mujoco_ee_grasp_pose_valid_)
        t -= (mujoco_ee_grasp_p_ - predicted_green);
    else
        t.z() -= base_world_z_;
    Eigen::Quaterniond q(msg->rotation.w, msg->rotation.x, msg->rotation.y, msg->rotation.z);
    if (q.norm() < 1e-9) q = Eigen::Quaterniond::Identity();
    q.normalize();
    pinocchio::SE3 T(q.toRotationMatrix(), t);
    ctrl_->set_ee_target(T);
}

// CM seed 또는 holding posture를 관절공간 posture reference로 넣는다.
void armPostureCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (!ctrl_ || msg->data.size() < 7) return;
    Eigen::VectorXd q(7);
    for (int i = 0; i < 7; ++i) q(i) = msg->data[i];
    ctrl_->set_arm_posture_target(q);
}

// elbow z 진단/실험용 reference. 0 이하이면 task를 제거한다.
void elbowZCallback(const std_msgs::Float32ConstPtr& msg) {
    if (!ctrl_) return;
    ctrl_->set_elbow_z_target(static_cast<double>(msg->data));
}

void baseTargetCallback(const geometry_msgs::Transform::ConstPtr& msg) {
    if (!ctrl_) return;
    Eigen::Quaterniond q(msg->rotation.w, msg->rotation.x, msg->rotation.y, msg->rotation.z);
    if (q.norm() < 1e-9) q = Eigen::Quaterniond::Identity();
    q.normalize();
    const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    const double yaw = std::atan2(siny_cosp, cosy_cosp);
    ctrl_->set_base_target(msg->translation.x, msg->translation.y, yaw);
}

// dual scene joint_states에서 현재 로봇 블록만 잘라 HQP에 전달한다.
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (static_cast<int>(msg->position.size()) < qpos_offset_ + kPosBlock) return;

    sensor_msgs::JointState m;
    m.position.resize(kPosBlock);
    m.velocity.resize(kVelBlock);
    m.effort.resize(kVelBlock);

    for (int i = 0; i < kPosBlock; ++i)
        m.position[i] = msg->position[qpos_offset_ + i];
    for (int i = 0; i < kVelBlock; ++i) {
        const int vi = qvel_offset_ + i;
        m.velocity[i] = (vi < static_cast<int>(msg->velocity.size())) ? msg->velocity[vi] : 0.0;
        m.effort[i]   = (vi < static_cast<int>(msg->effort.size()))   ? msg->effort[vi]   : 0.0;
    }

    ctrl_->franka_update(m);
    ctrl_->husky_update(m);
    base_x_ = m.position[0];
    base_world_z_ = m.position[2];
    for (int i = 0; i < 7; ++i) arm_q_[i] = m.position[11 + i];

    // Weld simulation에서 object-grasp/EE 오차를 spring wrench로 근사한다.
    if (fext_enabled_ && fext_source_ == "coupling" && couple_stiffness_ > 0.0 &&
        static_cast<int>(msg->position.size()) >= object_qpos_offset_ + 7) {
        const int o = object_qpos_offset_;
        Vector3d obj_p(msg->position[o + 0], msg->position[o + 1], msg->position[o + 2]);
        Quaterniond obj_q(msg->position[o + 3], msg->position[o + 4],
                          msg->position[o + 5], msg->position[o + 6]);
        if (!obj_p.allFinite() || !obj_q.coeffs().allFinite() || obj_q.norm() < 1e-9) {
            fext_sample_valid_ = false;
            ROS_WARN_THROTTLE(1.0, "[coop_husky_hqp] invalid object pose for coupling Fext");
            state_received_ = true;
            return;
        }
        obj_q.normalize();
        const Vector3d grasp_world = obj_p + obj_q * couple_grasp_offset_;

        pinocchio::SE3 link7_pose;
        ctrl_->position(link7_pose);
        Vector3d ee_world;
        if (mujoco_ee_grasp_pose_valid_) {
            ee_world = mujoco_ee_grasp_p_;
        } else {
            ee_world = link7_pose.translation() + link7_pose.rotation() * ee_grasp_offset_;
            ee_world.z() += base_world_z_;
        }

        Vector3d f = couple_stiffness_ * (grasp_world - ee_world);
        if (couple_force_limit_ > 0.0)
            for (int i = 0; i < 3; ++i)
                f(i) = std::max(-couple_force_limit_, std::min(couple_force_limit_, f(i)));

        Vector6d Fext;
        Fext << f(0), f(1), f(2), 0.0, 0.0, 0.0;
        updateFextSample(Fext);
    }

    state_received_ = true;
}

void publishEeGraspPose(const ros::Time& stamp) {
    if (!ctrl_ || ee_grasp_pose_pub_.getNumSubscribers() == 0)
        return;

    pinocchio::SE3 link7_pose;
    ctrl_->position(link7_pose);

    Vector3d p;
    Quaterniond q_link(link7_pose.rotation());
    q_link.normalize();
    Quaterniond q;
    if (mujoco_ee_grasp_pose_valid_) {
        p = mujoco_ee_grasp_p_;
        q = mujoco_ee_grasp_q_;
    } else {
        p = link7_pose.translation() + link7_pose.rotation() * ee_grasp_offset_;
        p.z() += base_world_z_;
        q = q_link * ee_grasp_quat_;
        q.normalize();
    }

    geometry_msgs::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "world";
    msg.child_frame_id = "ee_grasp_frame";
    msg.transform.translation.x = p.x();
    msg.transform.translation.y = p.y();
    msg.transform.translation.z = p.z();
    msg.transform.rotation.w = q.w();
    msg.transform.rotation.x = q.x();
    msg.transform.rotation.y = q.y();
    msg.transform.rotation.z = q.z();
    ee_grasp_pose_pub_.publish(msg);
}

void publishFramePose(const pinocchio::SE3& pose,
                      const ros::Time& stamp,
                      const string& child_frame_id,
                      ros::Publisher& pub) {
    if (pub.getNumSubscribers() == 0)
        return;

    Quaterniond q(pose.rotation());
    q.normalize();

    geometry_msgs::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "world";
    msg.child_frame_id = child_frame_id;
    msg.transform.translation.x = pose.translation().x();
    msg.transform.translation.y = pose.translation().y();
    msg.transform.translation.z = pose.translation().z();
    msg.transform.rotation.w = q.w();
    msg.transform.rotation.x = q.x();
    msg.transform.rotation.y = q.y();
    msg.transform.rotation.z = q.z();
    pub.publish(msg);
}

void publishArmDiagnosticPoses(const ros::Time& stamp) {
    if (!ctrl_)
        return;
    pinocchio::SE3 link7_pose;
    ctrl_->position(link7_pose);
    link7_pose.translation().z() += base_world_z_;
    publishFramePose(link7_pose, stamp, "panda_joint7", link7_pose_pub_);

    pinocchio::SE3 elbow_pose;
    ctrl_->elbow_position(elbow_pose);
    elbow_pose.translation().z() += base_world_z_;
    publishFramePose(elbow_pose, stamp, "panda_joint4", elbow_pose_pub_);
}

// MuJoCo EE force/torque sensor 값을 world frame Fext_로 변환한다.
void sensorStateCallback(const mujoco_ros_msgs::SensorState::ConstPtr& msg) {
    if (!ctrl_) return;
    const std::vector<double>* pdata = nullptr;
    const std::vector<double>* qdata = nullptr;
    const std::vector<double>* fdata = nullptr;
    const std::vector<double>* tdata = nullptr;
    for (const auto& s : msg->sensor) {
        if (s.name == ee_position_sensor_name_   && s.data.size() >= 3) pdata = &s.data;
        if (s.name == ee_quaternion_sensor_name_ && s.data.size() >= 4) qdata = &s.data;
        if (s.name == force_sensor_name_  && s.data.size() >= 3) fdata = &s.data;
        if (s.name == torque_sensor_name_ && s.data.size() >= 3) tdata = &s.data;
    }

    if (pdata != nullptr && qdata != nullptr) {
        const Vector3d p((*pdata)[0], (*pdata)[1], (*pdata)[2]);
        Quaterniond q((*qdata)[0], (*qdata)[1], (*qdata)[2], (*qdata)[3]);
        if (p.allFinite() && q.coeffs().allFinite() && q.norm() > 1e-9) {
            q.normalize();
            mujoco_ee_grasp_p_ = p;
            mujoco_ee_grasp_q_ = q;
            mujoco_ee_grasp_pose_valid_ = true;
        } else {
            mujoco_ee_grasp_pose_valid_ = false;
            ROS_WARN_THROTTLE(1.0, "[coop_husky_hqp] invalid MuJoCo EE grasp frame sample");
        }
    }

    if (!fext_enabled_ || fext_source_ != "sensor" || !state_received_) return;
    if (fdata == nullptr) return;  // force sensor is required; torque optional

    Vector3d f_site((*fdata)[0], (*fdata)[1], (*fdata)[2]);
    Vector3d t_site = Vector3d::Zero();
    if (tdata != nullptr) t_site = Vector3d((*tdata)[0], (*tdata)[1], (*tdata)[2]);
    if (!f_site.allFinite() || !t_site.allFinite()) {
        fext_sample_valid_ = false;
        ROS_WARN_THROTTLE(1.0, "[coop_husky_hqp] rejected non-finite F/T sensor sample");
        return;
    }

    // EE grasp frame의 world rotation.
    pinocchio::SE3 link7_pose;
    ctrl_->position(link7_pose);
    Quaterniond q_link(link7_pose.rotation());
    q_link.normalize();
    Quaterniond q_ee = q_link * ee_grasp_quat_;
    q_ee.normalize();
    const Matrix3d R_ee = q_ee.toRotationMatrix();

    const Vector3d f_world = ee_force_scale_  * (R_ee * f_site);
    const Vector3d t_world = ee_torque_scale_ * (R_ee * t_site);

    Vector6d Fext;
    Fext << f_world(0), f_world(1), f_world(2), t_world(0), t_world(1), t_world(2);
    updateFextSample(Fext);
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "coop_husky_hqp_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    string group_name, js_topic, st_topic, cmd_topic, mode_topic;
    string ee_target_topic, base_target_topic, ee_pose_topic, link7_pose_topic, elbow_pose_topic;
    int initial_mode;
    double rate_hz;
    pnh.param<string>("group_name", group_name, string("ns1"));
    pnh.param<string>("joint_states_topic", js_topic,
                      string("/coop_scene/mujoco_ros/mujoco_ros_interface/joint_states"));
    pnh.param<string>("sim_time_topic", st_topic,
                      string("/coop_scene/mujoco_ros/mujoco_ros_interface/sim_time"));
    pnh.param<string>("command_topic", cmd_topic, string("/leader/joint_command"));
    pnh.param<string>("ctrl_mode_topic", mode_topic, string("/coop/ctrl_mode"));
    pnh.param<string>("ee_target_topic", ee_target_topic, string("/leader/ee_target"));
    pnh.param<string>("base_target_topic", base_target_topic, string("/leader/base_target"));
    pnh.param<string>("ee_grasp_pose_topic", ee_pose_topic, string("/leader/ee_grasp_pose"));
    pnh.param<string>("link7_pose_topic", link7_pose_topic, string("/leader/link7_pose"));
    pnh.param<string>("elbow_pose_topic", elbow_pose_topic, string("/leader/elbow_pose"));
    pnh.param<int>("qpos_offset", qpos_offset_, 0);
    pnh.param<int>("qvel_offset", qvel_offset_, 0);
    pnh.param<int>("initial_ctrl_mode", initial_mode, 1);
    pnh.param<double>("rate", rate_hz, 1000.0);
    pnh.param<int>("cm_lift_mode", cm_lift_mode_, 238);
    pnh.param<int>("transport_mode", transport_mode_, 235);
    string cm_transport_ready_topic;
    pnh.param<string>("cm_transport_ready_topic", cm_transport_ready_topic, string("/coop/cm_lift_ready"));
    string sensor_state_topic;
    pnh.param<string>("sensor_state_topic", sensor_state_topic,
                      string("/coop_scene/mujoco_ros/mujoco_ros_interface/sensor_states"));
    pnh.param<bool>("enable_fext", fext_enabled_, false);
    pnh.param<string>("fext_source", fext_source_, string("sensor"));
    pnh.param<bool>("fext_comp_enabled", fext_comp_enabled_, false);
    pnh.param<double>("fext_comp_gain", fext_comp_gain_, 1.0);
    pnh.param<double>("fext_timeout", fext_timeout_, 0.25);
    // coupling source
    pnh.param<int>("object_qpos_offset", object_qpos_offset_, 40);
    pnh.param<double>("couple_stiffness", couple_stiffness_, 0.0);
    pnh.param<double>("couple_force_limit", couple_force_limit_, 0.0);
    readVector3Param(pnh, "couple_grasp_offset_xyz", couple_grasp_offset_);
    // sensor source
    pnh.param<double>("ee_force_scale", ee_force_scale_, 1.0);
    pnh.param<double>("ee_torque_scale", ee_torque_scale_, 0.0);
    pnh.param<string>("ee_force_sensor_name", force_sensor_name_, string("leader_ee_force"));
    pnh.param<string>("ee_torque_sensor_name", torque_sensor_name_, string("leader_ee_torque"));
    pnh.param<string>("ee_position_sensor_name", ee_position_sensor_name_,
                      string("leader_ee_grasp_position"));
    pnh.param<string>("ee_quaternion_sensor_name", ee_quaternion_sensor_name_,
                      string("leader_ee_grasp_quaternion"));

    bool issimulation = true, isrobotiq = false;
    pnh.param<bool>("issimulation", issimulation, true);
    pnh.param<bool>("robotiq_gripper", isrobotiq, false);
    readVector3Param(pnh, "ee_grasp_offset_xyz", ee_grasp_offset_);
    readQuaternionParam(pnh, "ee_grasp_offset_quat_wxyz", ee_grasp_quat_);

    if (qpos_offset_ < 0 || qvel_offset_ < 0 || object_qpos_offset_ < 0) {
        ROS_FATAL("[coop_husky_hqp] offsets must be non-negative (qpos=%d qvel=%d object=%d)",
                  qpos_offset_, qvel_offset_, object_qpos_offset_);
        return 2;
    }
    if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
        ROS_FATAL("[coop_husky_hqp] rate must be finite and positive");
        return 2;
    }
    if (fext_enabled_ && fext_source_ != "sensor" && fext_source_ != "coupling") {
        ROS_FATAL("[coop_husky_hqp] fext_source must be 'sensor' or 'coupling', got '%s'",
                  fext_source_.c_str());
        return 2;
    }
    if (fext_comp_enabled_ && !fext_enabled_) {
        ROS_FATAL("[coop_husky_hqp] fext_comp_enabled requires enable_fext=true");
        return 2;
    }
    if (fext_enabled_ && fext_source_ == "coupling" &&
        (!std::isfinite(couple_stiffness_) || couple_stiffness_ <= 0.0)) {
        ROS_FATAL("[coop_husky_hqp] coupling Fext requires couple_stiffness > 0");
        return 2;
    }
    if (!std::isfinite(fext_comp_gain_) || fext_comp_gain_ < 0.0 ||
        !std::isfinite(fext_timeout_) || fext_timeout_ <= 0.0 ||
        !std::isfinite(ee_force_scale_) || !std::isfinite(ee_torque_scale_)) {
        ROS_FATAL("[coop_husky_hqp] invalid Fext gain/timeout/scale parameter");
        return 2;
    }

    // 로봇별 URDF 경로는 launch의 /<group_name>/robot_urdf* 파라미터를 사용한다.
    ctrl_ = new RobotController::FrankaHuskyWrapper(
        group_name, issimulation, true, isrobotiq, nh);
    ctrl_->initialize();

    command_msg_.torque.resize(13);

    ros::Subscriber js_sub =
        nh.subscribe(js_topic, 5, jointStateCallback, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber st_sub = nh.subscribe(st_topic, 1, simTimeCallback);
    ros::Subscriber mode_sub = nh.subscribe(mode_topic, 1, ctrlModeCallback);
    ros::Subscriber cm_transport_ready_sub =
        nh.subscribe(cm_transport_ready_topic, 1, cmTransportReadyCallback);
    ros::Subscriber ee_target_sub = nh.subscribe(ee_target_topic, 1, eeTargetCallback);
    ros::Subscriber base_target_sub = nh.subscribe(base_target_topic, 1, baseTargetCallback);
    string arm_posture_topic;
    pnh.param<string>("arm_posture_target_topic", arm_posture_topic, string(""));
    ros::Subscriber arm_posture_sub;
    if (!arm_posture_topic.empty())
        arm_posture_sub = nh.subscribe(arm_posture_topic, 1, armPostureCallback);
    string elbow_z_topic;
    pnh.param<string>("elbow_z_target_topic", elbow_z_topic, string(""));
    ros::Subscriber elbow_z_sub;
    if (!elbow_z_topic.empty())
        elbow_z_sub = nh.subscribe(elbow_z_topic, 1, elbowZCallback);
    ros::Subscriber sensor_sub =
        nh.subscribe(sensor_state_topic, 1, sensorStateCallback,
                     ros::TransportHints().tcpNoDelay(true));
    ROS_INFO("[coop_husky_hqp] exact MuJoCo grasp pose %s (position=%s quaternion=%s)",
             sensor_state_topic.c_str(), ee_position_sensor_name_.c_str(),
             ee_quaternion_sensor_name_.c_str());
    if (fext_enabled_ && fext_source_ == "sensor") {
        ROS_INFO("[coop_husky_hqp] Fext source=sensor %s (force=%s torque=%s scale f=%.3f t=%.3f)",
                 sensor_state_topic.c_str(), force_sensor_name_.c_str(),
                 torque_sensor_name_.c_str(), ee_force_scale_, ee_torque_scale_);
    } else if (fext_enabled_ && fext_source_ == "coupling") {
        ROS_INFO("[coop_husky_hqp] Fext source=coupling k=%.1f N/m grasp_off=[%.2f %.2f %.2f] obj_qpos_off=%d limit=%.1f",
                 couple_stiffness_, couple_grasp_offset_(0), couple_grasp_offset_(1),
                 couple_grasp_offset_(2), object_qpos_offset_, couple_force_limit_);
    }
    command_pub_ = nh.advertise<mujoco_ros_msgs::JointSet>(cmd_topic, 5);
    ee_grasp_pose_pub_ = nh.advertise<geometry_msgs::TransformStamped>(ee_pose_topic, 5);
    link7_pose_pub_ = nh.advertise<geometry_msgs::TransformStamped>(link7_pose_topic, 5);
    elbow_pose_pub_ = nh.advertise<geometry_msgs::TransformStamped>(elbow_pose_topic, 5);

    ROS_INFO("[coop_husky_hqp] group=%s qpos_off=%d qvel_off=%d cmd=%s mode=%d rate=%.0f",
             group_name.c_str(), qpos_offset_, qvel_offset_, cmd_topic.c_str(), initial_mode, rate_hz);

    // 첫 joint state 이후 initial mode를 적용해 유효한 자세에서 기준 pose를 잡는다.
    ros::Rate loop_rate(rate_hz);
    bool initialized = false;
    while (ros::ok()) {
        ros::spinOnce();

        if (state_received_) {
            if (!initialized) {
                const int startup_mode = pending_mode_valid_ ? pending_mode_ : initial_mode;
                ctrl_->ctrl_update(startup_mode);
                cur_hqp_mode_ = startup_mode;
                controller_initialized_ = true;
                pending_mode_valid_ = false;
                initialized = true;
            }

            ctrl_->compute(time_);

            ctrl_->mass(robot_mass_);
            ctrl_->nle(robot_nle_);
            ctrl_->franka_output(franka_qacc_);                  // arm joint accel (7)
            franka_torque_ = robot_mass_ * franka_qacc_ + robot_nle_;  // computed torque (7)
            // 선택 사항: 실제 gripper/FT sensor 또는 weld simulation wrench 하중 보상.
            if (fext_comp_enabled_ && cur_hqp_mode_ == 235) {
                if (fextSampleFresh()) {
                    VectorXd tau_fext;
                    ctrl_->Fext_comp_arm_torque(tau_fext);        // JWorld^T * Fext_ (7)
                    if (tau_fext.size() == franka_torque_.size() && tau_fext.allFinite())
                        franka_torque_ -= fext_comp_gain_ * tau_fext;
                } else {
                    ROS_WARN_THROTTLE(1.0, "[coop_husky_hqp] Fext sample missing/stale; compensation skipped");
                }
            }
            ctrl_->husky_output(husky_qacc_);     // wheel velocity cmd (2)

            command_msg_.MODE = 1;  // torque control
            command_msg_.header.stamp = ros::Time::now();
            command_msg_.time = time_;
            for (int i = 0; i < 13; ++i) command_msg_.torque[i] = 0.0;

            command_msg_.torque[0] = husky_qacc_(0);  // front-left  wheel
            command_msg_.torque[2] = husky_qacc_(0);  // rear-left   wheel
            command_msg_.torque[1] = husky_qacc_(1);  // front-right wheel
            command_msg_.torque[3] = husky_qacc_(1);  // rear-right  wheel
            for (int i = 0; i < 7; ++i) command_msg_.torque[i + 4] = franka_torque_(i);
            // Slots 11/12 are reserved for the real gripper command path. The
            // current weld backend is simulation-only and leaves them at zero.

            const bool command_finite = std::all_of(
                command_msg_.torque.begin(), command_msg_.torque.end(),
                [](double value) { return std::isfinite(value); });
            if (!command_finite) {
                std::fill(command_msg_.torque.begin(), command_msg_.torque.end(), 0.0);
                ROS_ERROR_THROTTLE(
                    1.0,
                    "[coop_husky_hqp] non-finite controller output rejected; publishing zero command");
            }

            command_pub_.publish(command_msg_);
            publishEeGraspPose(command_msg_.header.stamp);
            publishArmDiagnosticPoses(command_msg_.header.stamp);

            // posture reference가 EE 높이를 유지하며 elbow를 낮추는지 확인하는 진단 로그.
            {
                pinocchio::SE3 ee_p, elb_p;
                ctrl_->position(ee_p);
                ctrl_->elbow_position(elb_p);
                ROS_DEBUG_THROTTLE(2.0,
                    "[coop_husky_hqp] %s base_x=%.2f EEz=%.3f elbowZ=%.3f q=[%.3f %.3f %.3f %.3f %.3f %.3f %.3f] (ceil 0.75)",
                    group_name.c_str(), base_x_, ee_p.translation().z(), elb_p.translation().z(),
                    arm_q_[0], arm_q_[1], arm_q_[2], arm_q_[3], arm_q_[4], arm_q_[5], arm_q_[6]);
            }
        }

        loop_rate.sleep();
    }

    delete ctrl_;
    return 0;
}
