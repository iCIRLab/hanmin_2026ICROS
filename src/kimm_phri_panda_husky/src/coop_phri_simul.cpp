// coop_phri_simul.cpp
//
// Cooperative transport support runtime. The two coop_husky_hqp_node processes
// own robot control; this single ROS process owns five supporting components:
// simulation/weld lifecycle, grasp metrics, object-centered targets, command
// merge, and CM lift readiness. Keeping them in one file/process is intentional.
// The weld path is simulation-only; the reserved gripper command path remains
// separate for future real-gripper transport.

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "XmlRpcValue.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "mujoco_ros_msgs/JointSet.h"
#include "ros/ros.h"
#include "ros/timer.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/String.h"
#include <yaml-cpp/yaml.h>

namespace {

template <int Size>
bool readVectorParam(ros::NodeHandle& pnh, const std::string& name,
                     Eigen::Matrix<double, Size, 1>& out) {
    std::vector<double> values;
    if (!pnh.getParam(name, values) || values.size() != Size)
        return false;
    for (int i = 0; i < Size; ++i)
        out(i) = values[i];
    return true;
}

Eigen::Quaterniond normalizedQuaternion(Eigen::Quaterniond q) {
    if (q.norm() < 1e-9)
        q = Eigen::Quaterniond::Identity();
    q.normalize();
    return q;
}

bool readQuaternionParam(ros::NodeHandle& pnh, const std::string& name,
                         Eigen::Quaterniond& out) {
    std::vector<double> values;
    if (!pnh.getParam(name, values) || values.size() != 4)
        return false;
    out = normalizedQuaternion(
        Eigen::Quaterniond(values[0], values[1], values[2], values[3]));
    return true;
}

// -----------------------------------------------------------------------------
// 1/5 Simulation lifecycle and fixed-weld test backend
// -----------------------------------------------------------------------------
// This is not a contact grasp controller. It toggles MuJoCo equality welds
// through kimm_mujoco_ros and performs the simulator RESET/INIT handshake.
//
// HQP reference refresh after attach:
//   When weld attach is confirmed by MuJoCo (EQ_ACTIVE_OK for every weld), the
//   upstream HQP reference can be stale. We schedule a one-shot ctrl_mode after
//   a short settle delay. The current runtime default is ctrl_mode=200, which
//   re-snapshots an external-EE-target HQP reference after the weld settles
//   while keeping the mobile base held. Set post_attach_refresh_mode to 0 to
//   disable it. The cooperative HQP accepts only modes 1, 200, and 235.

class CoopGraspTransitionNode {
public:
    CoopGraspTransitionNode() : pnh_("~") {
        pnh_.param<std::string>("ctrl_mode_topic", ctrl_mode_topic_, "/coop/ctrl_mode");
        pnh_.param<std::string>("sim_command_topic", sim_command_topic_,
                                "/coop_scene/mujoco_ros/mujoco_ros_interface/sim_command_con2sim");
        pnh_.param<std::string>("sim_status_topic", sim_status_topic_,
                                "/coop_scene/mujoco_ros/mujoco_ros_interface/sim_command_sim2con");
        pnh_.param<std::string>("state_topic", state_topic_, "/coop/grasp_state");
        pnh_.param<std::string>("gap_topic", gap_topic_, "/coop/attach_gap");
        pnh_.param<std::string>("sim_run_topic", sim_run_topic_,
                                "/coop_scene/mujoco_ros/mujoco_ros_interface/sim_run");
        pnh_.param("attach_mode", attach_mode_, 21);
        pnh_.param("pregrasp_mode", pregrasp_mode_, 24);
        pnh_.param("initial_attached", initial_attached_, false);
        // Mode republished to refresh HQP reference after attach settles.
        // 0 disables. Default 200 keeps an external-EE-target HQP reference;
        // modes 2/210 can be supplied from launch for comparison.
        pnh_.param("post_attach_refresh_mode", post_attach_refresh_mode_, 200);
        // Settle delay before the refresh. MuJoCo weld solref time constant is
        // 0.05 s; ~4x gives a conservative settle margin.
        pnh_.param("post_attach_refresh_delay", post_attach_refresh_delay_, 0.2);
        pnh_.param("attach_request_delay", attach_request_delay_, 3.0);
        pnh_.param("enable_attach_gap_gate", enable_attach_gap_gate_, true);
        pnh_.param("max_attach_gap", max_attach_gap_, 0.20);
        pnh_.param("gap_timeout", gap_timeout_, 1.0);
        pnh_.param("start_sim_after_attach", start_sim_after_attach_, false);
        // Run the sim from launch (latched sim_run=true) so the pre-grasp
        // approach (mode 24, key 'p') can actually move the arms BEFORE the weld.
        // The robots hold their folded spawn pose (HQP) and the plate rests on
        // its start supports, so nothing drifts while idle.
        pnh_.param("start_sim_on_init", start_sim_on_init_, false);
        pnh_.param("start_sim_on_first_mode", start_sim_on_first_mode_, false);
        XmlRpc::XmlRpcValue ignore_modes_param;
        if (pnh_.getParam("start_sim_ignore_modes", ignore_modes_param) &&
            ignore_modes_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int i = 0; i < ignore_modes_param.size(); ++i) {
                if (ignore_modes_param[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
                    start_sim_ignore_modes_.push_back(static_cast<int>(ignore_modes_param[i]));
            }
        }

        welds_.push_back("object_to_leader_ee_fixed");
        welds_.push_back("object_to_follower_ee_fixed");
        XmlRpc::XmlRpcValue weld_param;
        if (pnh_.getParam("welds", weld_param) && weld_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            welds_.clear();
            for (int i = 0; i < weld_param.size(); ++i)
                welds_.push_back(static_cast<std::string>(weld_param[i]));
        }
        weld_active_.assign(welds_.size(), initial_attached_);

        command_pub_ = nh_.advertise<std_msgs::String>(sim_command_topic_, 10);
        sim_run_pub_ = nh_.advertise<std_msgs::Bool>(sim_run_topic_, 1, true);
        state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 1, true);
        // Republish mode 200 after attach so both HQP controllers replan from the
        // settled welded pose. This component ignores mode 200 itself.
        ctrl_mode_pub_ = nh_.advertise<std_msgs::Int32>(ctrl_mode_topic_, 1);
        mode_sub_ = nh_.subscribe(ctrl_mode_topic_, 1, &CoopGraspTransitionNode::modeCallback, this);
        status_sub_ = nh_.subscribe(sim_status_topic_, 10, &CoopGraspTransitionNode::statusCallback, this);
        gap_sub_ = nh_.subscribe(gap_topic_, 1, &CoopGraspTransitionNode::gapCallback, this);

        publishState(initial_attached_ ? "attached" : "detached");
        publishSimRun(start_sim_on_init_, start_sim_on_init_ ? "start_sim_on_init" : "hold_before_attach");
        ROS_INFO("[coop_grasp_transition_node] ctrl_mode=%s sim_command=%s welds=%zu gap_gate=%s max_gap=%.3f start_sim_after_attach=%s start_sim_on_init=%s start_sim_on_first_mode=%s ignore_modes=%s",
                 ctrl_mode_topic_.c_str(), sim_command_topic_.c_str(), welds_.size(),
                 enable_attach_gap_gate_ ? "true" : "false", max_attach_gap_,
                 start_sim_after_attach_ ? "true" : "false",
                 start_sim_on_init_ ? "true" : "false",
                 start_sim_on_first_mode_ ? "true" : "false",
                 modeListText(start_sim_ignore_modes_).c_str());
    }

private:
    std::string modeListText(const std::vector<int>& modes) const {
        std::stringstream ss;
        ss << "[";
        for (size_t i = 0; i < modes.size(); ++i) {
            if (i > 0)
                ss << ",";
            ss << modes[i];
        }
        ss << "]";
        return ss.str();
    }

    bool modeIgnoredForSimStart(int mode) const {
        return std::find(start_sim_ignore_modes_.begin(), start_sim_ignore_modes_.end(), mode) !=
               start_sim_ignore_modes_.end();
    }

    void publishState(const std::string& state) {
        std_msgs::String msg;
        msg.data = state;
        state_pub_.publish(msg);
        ROS_INFO("[coop_grasp_transition_node] grasp_state=%s", state.c_str());
    }

    void requestWeldState(bool active) {
        const bool all_active_now = allWeldsActive();
        const bool any_active_now = anyWeldActive();
        if (active && all_active_now) {
            publishState("attached");
            ROS_INFO_THROTTLE(1.0, "[coop_grasp_transition_node] attach ignored: welds already active");
            return;
        }
        if (!active && !any_active_now) {
            publishState("detached");
            ROS_INFO_THROTTLE(1.0, "[coop_grasp_transition_node] detach ignored: welds already inactive");
            return;
        }
        if (active && !attachGapAcceptable())
            return;
        pending_attach_on_gap_ = false;
        pending_state_ = active ? "attached" : "detached";
        publishState(active ? "attach_requested" : "detach_requested");
        for (const std::string& weld : welds_) {
            std_msgs::String cmd;
            cmd.data = std::string("EQ_ACTIVE ") + weld + (active ? " 1" : " 0");
            command_pub_.publish(cmd);
        }
    }

    void modeCallback(const std_msgs::Int32ConstPtr& msg) {
        const int mode = msg->data;
        maybeStartSimOnFirstMode(mode);
        if (mode == pregrasp_mode_) {
            publishState("pregrasp_ready");
        } else if (mode == attach_mode_) {
            ROS_WARN("[coop_grasp_transition_node] requesting fixed-weld attach; this is not contact grasp");
            if (attach_request_delay_ > 0.0) {
                publishState("attach_waiting");
                attach_timer_ = nh_.createTimer(
                    ros::Duration(attach_request_delay_),
                    &CoopGraspTransitionNode::publishDelayedAttach, this,
                    /*oneshot=*/true);
                ROS_INFO("[coop_grasp_transition_node] delayed attach scheduled: %.2f s",
                         attach_request_delay_);
            } else {
                requestWeldState(true);
            }
        }
    }

    void statusCallback(const std_msgs::StringConstPtr& msg) {
        // kimm_mujoco_ros expects the external controller side to echo RESET and
        // INIT on sim_command_con2sim during connector startup. In the old single
        // robot runtime phri_simul.cpp performed this handshake. The C++ dual
        // runtime does not run phri_simul, so this node owns the lightweight echo
        // together with weld attach/detach commands.
        if (msg->data == "RESET" || msg->data == "INIT") {
            std_msgs::String reply;
            reply.data = msg->data;
            command_pub_.publish(reply);
            ROS_INFO("[coop_grasp_transition_node] echoed MuJoCo startup handshake: %s",
                     msg->data.c_str());
            return;
        }

        std::stringstream ss(msg->data);
        std::string tag, name, active_string;
        ss >> tag >> name >> active_string;
        if (tag != "EQ_ACTIVE_OK" || name.empty() || active_string.empty())
            return;
        for (size_t i = 0; i < welds_.size(); ++i) {
            if (welds_[i] == name)
                weld_active_[i] = (active_string == "1");
        }

        if (pending_state_ == "attached" && allWeldsActive()) {
            pending_state_.clear();
            publishState("attached");
            publishSimRunIfRequested();
            scheduleHqpReferenceRefresh();
        } else if (pending_state_ == "detached" && !anyWeldActive()) {
            pending_state_.clear();
            publishState("detached");
        }
    }

    bool allWeldsActive() const {
        return !weld_active_.empty() &&
               std::all_of(weld_active_.begin(), weld_active_.end(), [](bool active) { return active; });
    }

    bool anyWeldActive() const {
        return std::any_of(weld_active_.begin(), weld_active_.end(), [](bool active) { return active; });
    }

    void gapCallback(const std_msgs::Float32MultiArrayConstPtr& msg) {
        double max_gap = 0.0;
        for (float v : msg->data) {
            const double gap = std::abs(static_cast<double>(v));
            if (gap > max_gap)
                max_gap = gap;
        }
        last_max_gap_ = max_gap;
        last_gap_stamp_ = ros::Time::now();

        if (pending_attach_on_gap_ && max_gap <= max_attach_gap_) {
            ROS_INFO("[coop_grasp_transition_node] delayed attach gap accepted: max_gap=%.4f m",
                     max_gap);
            pending_attach_on_gap_ = false;
            requestWeldState(true);
        }
    }

    bool attachGapAcceptable() {
        if (last_gap_stamp_.isZero()) {
            if (enable_attach_gap_gate_) {
                ROS_WARN("[coop_grasp_transition_node] attach waiting: no fresh EE-object gap on %s",
                         gap_topic_.c_str());
                pending_attach_on_gap_ = true;
                publishState("attach_waiting_gap");
                return false;
            }
            ROS_WARN("[coop_grasp_transition_node] attach without runtime EE-object gap check; "
                     "publish %s and enable enable_attach_gap_gate for hard gating",
                     gap_topic_.c_str());
            return true;
        }

        const double age = (ros::Time::now() - last_gap_stamp_).toSec();
        if (age > gap_timeout_) {
            if (enable_attach_gap_gate_) {
                ROS_WARN("[coop_grasp_transition_node] attach rejected: stale EE-object gap age=%.3f s",
                         age);
                publishState("attach_rejected_stale_gap");
                return false;
            }
            ROS_WARN("[coop_grasp_transition_node] attach using stale/unverified gap age=%.3f s max_gap=%.4f m",
                     age, last_max_gap_);
            return true;
        }

        if (last_max_gap_ > max_attach_gap_) {
            if (enable_attach_gap_gate_) {
                ROS_WARN("[coop_grasp_transition_node] attach rejected: max EE-object gap %.4f m > %.4f m",
                         last_max_gap_, max_attach_gap_);
                publishState("attach_rejected_gap");
                return false;
            }
            ROS_WARN("[coop_grasp_transition_node] attach requested with large EE-object gap %.4f m > %.4f m "
                     "(gate disabled)",
                     last_max_gap_, max_attach_gap_);
            return true;
        }

        ROS_INFO("[coop_grasp_transition_node] attach gap accepted: max_gap=%.4f m age=%.3f s",
                 last_max_gap_, age);
        return true;
    }

    // After the weld is confirmed active and the constraint has had time to
    // settle, republish ctrl_mode so either the HQP or an orchestrator node can
    // refresh references against the attached pose.
    void scheduleHqpReferenceRefresh() {
        if (post_attach_refresh_mode_ == 0)
            return;
        refresh_timer_ = nh_.createTimer(
            ros::Duration(post_attach_refresh_delay_),
            &CoopGraspTransitionNode::publishHqpReferenceRefresh, this,
            /*oneshot=*/true);
    }

    void publishHqpReferenceRefresh(const ros::TimerEvent&) {
        std_msgs::Int32 msg;
        msg.data = post_attach_refresh_mode_;
        ctrl_mode_pub_.publish(msg);
        ROS_INFO("[coop_grasp_transition_node] post-attach HQP reference refresh: ctrl_mode=%d",
                 post_attach_refresh_mode_);
    }

    void publishSimRun(bool enabled, const char* reason) {
        std_msgs::Bool run;
        run.data = enabled;
        sim_run_pub_.publish(run);
        sim_run_started_ = enabled;
        ROS_INFO("[coop_grasp_transition_node] published sim_run=%s (%s)",
                 enabled ? "true" : "false", reason);
    }

    void maybeStartSimOnFirstMode(int mode) {
        if (!start_sim_on_first_mode_ || sim_run_started_)
            return;
        if (modeIgnoredForSimStart(mode)) {
            ROS_INFO("[coop_grasp_transition_node] first mode %d did not start MuJoCo simulation (ignored)", mode);
            return;
        }
        publishSimRun(true, "first_mode");
        ROS_INFO("[coop_grasp_transition_node] first mode %d started MuJoCo simulation", mode);
    }

    void publishSimRunIfRequested() {
        if (!start_sim_after_attach_)
            return;
        publishSimRun(true, "after_attach");
    }

    void publishDelayedAttach(const ros::TimerEvent&) {
        requestWeldState(true);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher command_pub_;
    ros::Publisher sim_run_pub_;
    ros::Publisher state_pub_;
    ros::Publisher ctrl_mode_pub_;
    ros::Subscriber mode_sub_;
    ros::Subscriber status_sub_;
    ros::Subscriber gap_sub_;
    ros::Timer refresh_timer_;
    ros::Timer attach_timer_;

    std::string ctrl_mode_topic_;
    std::string sim_command_topic_;
    std::string sim_status_topic_;
    std::string state_topic_;
    std::string gap_topic_;
    std::string sim_run_topic_;
    int attach_mode_ = 21;
    int pregrasp_mode_ = 24;
    int post_attach_refresh_mode_ = 200;
    double post_attach_refresh_delay_ = 0.2;
    double attach_request_delay_ = 3.0;
    double max_attach_gap_ = 0.20;
    double gap_timeout_ = 1.0;
    bool initial_attached_ = false;
    bool enable_attach_gap_gate_ = true;
    bool start_sim_after_attach_ = false;
    bool start_sim_on_init_ = false;
    bool start_sim_on_first_mode_ = false;
    bool sim_run_started_ = false;
    bool pending_attach_on_gap_ = false;
    std::vector<int> start_sim_ignore_modes_;
    std::vector<std::string> welds_;
    std::vector<bool> weld_active_;
    double last_max_gap_ = 0.0;
    ros::Time last_gap_stamp_;
    std::string pending_state_;
};

// -----------------------------------------------------------------------------
// 2/5 Grasp transform and object stability monitor
// -----------------------------------------------------------------------------
// Runtime consistency monitor for the fixed-weld cooperative transport demo.
// It compares the two MuJoCo object grasp frames with the two HQP-published EE
// grasp poses before attach, then keeps reporting object tilt and the relative
// z heights of the object center / left grasp / right grasp during lift. It also
// reports object x/y and xy drift to catch cases where the plate looks stable in
// height but is being dragged laterally after fixed-weld attach.
struct Pose {
    Eigen::Vector3d p{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
    ros::Time stamp;
    bool valid{false};
};

double clampUnit(double v) {
    return std::max(-1.0, std::min(1.0, v));
}

double orientationError(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
    const double dot = std::abs(a.normalized().dot(b.normalized()));
    return 2.0 * std::acos(clampUnit(dot));
}

class CoopGraspTfMonitor {
public:
    CoopGraspTfMonitor() : pnh_("~") {
        pnh_.param<std::string>("joint_states_topic", joint_states_topic_,
                                "/coop_scene/mujoco_ros/mujoco_ros_interface/joint_states");
        pnh_.param<std::string>("leader_ee_pose_topic", leader_ee_pose_topic_, "/leader/ee_grasp_pose");
        pnh_.param<std::string>("follower_ee_pose_topic", follower_ee_pose_topic_, "/follower/ee_grasp_pose");
        pnh_.param<std::string>("leader_link7_pose_topic", leader_link7_pose_topic_, "/leader/link7_pose");
        pnh_.param<std::string>("follower_link7_pose_topic", follower_link7_pose_topic_, "/follower/link7_pose");
        pnh_.param<std::string>("leader_elbow_pose_topic", leader_elbow_pose_topic_, "/leader/elbow_pose");
        pnh_.param<std::string>("follower_elbow_pose_topic", follower_elbow_pose_topic_, "/follower/elbow_pose");
        pnh_.param<std::string>("attach_gap_topic", attach_gap_topic_, "/coop/attach_gap");
        pnh_.param<std::string>("grasp_tf_error_topic", grasp_tf_error_topic_, "/coop/grasp_tf_error");
        pnh_.param<int>("object_qpos_offset", object_qpos_offset_, 40);
        pnh_.param<double>("max_pose_age", max_pose_age_, 0.5);
        pnh_.param<double>("monitor_rate", rate_hz_, 50.0);
        pnh_.param<double>("object_keepout_margin", object_keepout_margin_, 0.03);
        double object_half_x = object_half_extents_.x();
        double object_half_y = object_half_extents_.y();
        double object_half_z = object_half_extents_.z();
        pnh_.param<double>("object_half_x", object_half_x, 0.267);
        pnh_.param<double>("object_half_y", object_half_y, 0.85);
        pnh_.param<double>("object_half_z", object_half_z, 0.025);
        object_half_extents_ = Eigen::Vector3d(object_half_x, object_half_y, object_half_z);
        readVectorParam(pnh_, "object_half_extents_xyz", object_half_extents_);
        readVectorParam(pnh_, "leader_object_grasp_offset_xyz", leader_object_grasp_offset_);
        readVectorParam(pnh_, "follower_object_grasp_offset_xyz", follower_object_grasp_offset_);
        readQuaternionParam(pnh_, "leader_object_grasp_offset_quat_wxyz", leader_object_grasp_quat_);
        readQuaternionParam(pnh_, "follower_object_grasp_offset_quat_wxyz", follower_object_grasp_quat_);

        js_sub_ = nh_.subscribe(joint_states_topic_, 5, &CoopGraspTfMonitor::jointStateCallback, this,
                               ros::TransportHints().tcpNoDelay(true));
        leader_sub_ = nh_.subscribe(leader_ee_pose_topic_, 5, &CoopGraspTfMonitor::leaderEeCallback, this);
        follower_sub_ = nh_.subscribe(follower_ee_pose_topic_, 5, &CoopGraspTfMonitor::followerEeCallback, this);
        leader_link7_sub_ = nh_.subscribe(leader_link7_pose_topic_, 5, &CoopGraspTfMonitor::leaderLink7Callback, this);
        follower_link7_sub_ = nh_.subscribe(follower_link7_pose_topic_, 5, &CoopGraspTfMonitor::followerLink7Callback, this);
        leader_elbow_sub_ = nh_.subscribe(leader_elbow_pose_topic_, 5, &CoopGraspTfMonitor::leaderElbowCallback, this);
        follower_elbow_sub_ = nh_.subscribe(follower_elbow_pose_topic_, 5, &CoopGraspTfMonitor::followerElbowCallback, this);
        gap_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(attach_gap_topic_, 5);
        error_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(grasp_tf_error_topic_, 5);

        ROS_INFO("[coop_grasp_tf_monitor] object_qpos_offset=%d object_half=[%.3f %.3f %.3f] keepout=%.3f leader_offset=[%.3f %.3f %.3f] follower_offset=[%.3f %.3f %.3f]",
                 object_qpos_offset_,
                 object_half_extents_.x(), object_half_extents_.y(), object_half_extents_.z(), object_keepout_margin_,
                 leader_object_grasp_offset_.x(), leader_object_grasp_offset_.y(), leader_object_grasp_offset_.z(),
                 follower_object_grasp_offset_.x(), follower_object_grasp_offset_.y(), follower_object_grasp_offset_.z());
    }

    void update() {
        publishMetrics();
    }

    double rateHz() const {
        return rate_hz_;
    }

private:
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        if (static_cast<int>(msg->position.size()) < object_qpos_offset_ + 7)
            return;

        object_pose_.p = Eigen::Vector3d(msg->position[object_qpos_offset_ + 0],
                                         msg->position[object_qpos_offset_ + 1],
                                         msg->position[object_qpos_offset_ + 2]);
        object_pose_.q = normalizedQuaternion(Eigen::Quaterniond(
            msg->position[object_qpos_offset_ + 3], msg->position[object_qpos_offset_ + 4],
            msg->position[object_qpos_offset_ + 5], msg->position[object_qpos_offset_ + 6]));
        object_pose_.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        object_pose_.valid = true;
        if (!initial_object_xy_valid_) {
            initial_object_xy_ = object_pose_.p.head<2>();
            initial_object_xy_valid_ = true;
        }
    }

    void setEePose(const geometry_msgs::TransformStamped::ConstPtr& msg, Pose& pose) {
        pose.p = Eigen::Vector3d(msg->transform.translation.x,
                                 msg->transform.translation.y,
                                 msg->transform.translation.z);
        pose.q = normalizedQuaternion(Eigen::Quaterniond(
            msg->transform.rotation.w, msg->transform.rotation.x,
            msg->transform.rotation.y, msg->transform.rotation.z));
        pose.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        pose.valid = true;
    }

    void leaderEeCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
        setEePose(msg, leader_ee_pose_);
    }

    void followerEeCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
        setEePose(msg, follower_ee_pose_);
    }

    void leaderLink7Callback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
        setEePose(msg, leader_link7_pose_);
    }

    void followerLink7Callback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
        setEePose(msg, follower_link7_pose_);
    }

    void leaderElbowCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
        setEePose(msg, leader_elbow_pose_);
    }

    void followerElbowCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
        setEePose(msg, follower_elbow_pose_);
    }

    Pose objectGraspPose(const Eigen::Vector3d& local_offset,
                         const Eigen::Quaterniond& local_quat) const {
        Pose out;
        out.p = object_pose_.p + object_pose_.q.toRotationMatrix() * local_offset;
        out.q = normalizedQuaternion(object_pose_.q * local_quat);
        out.stamp = object_pose_.stamp;
        out.valid = object_pose_.valid;
        return out;
    }

    void objectRpy(double& roll, double& pitch, double& yaw) const {
        const Eigen::Matrix3d R = object_pose_.q.toRotationMatrix();
        roll = std::atan2(R(2, 1), R(2, 2));
        pitch = std::asin(clampUnit(-R(2, 0)));
        yaw = std::atan2(R(1, 0), R(0, 0));
    }

    bool fresh(const Pose& pose, const ros::Time& now) const {
        if (!pose.valid)
            return false;
        return (now - pose.stamp).toSec() <= max_pose_age_;
    }

    Eigen::Vector3d objectFramePoint(const Eigen::Vector3d& world_point) const {
        return object_pose_.q.conjugate() * (world_point - object_pose_.p);
    }

    struct ClearanceSample {
        Eigen::Vector3d local{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
        double side_clearance{std::numeric_limits<double>::quiet_NaN()};
        double penetration_depth{std::numeric_limits<double>::quiet_NaN()};
        double penetration_flag{std::numeric_limits<double>::quiet_NaN()};
        double keepout_violation{std::numeric_limits<double>::quiet_NaN()};
        bool valid{false};
    };

    ClearanceSample followerSideClearance(const Pose& pose, const ros::Time& now) const {
        ClearanceSample out;
        if (!fresh(pose, now))
            return out;
        out.valid = true;
        out.local = objectFramePoint(pose.p);
        // Follower should remain outside the negative-y object face. Positive is safe;
        // negative means the sampled point crossed inward past the object side plane.
        // That projected side-plane violation is not a physical penetration when the
        // point is above/below or before/after the finite box, so check all three
        // object-frame axes before setting penetration_depth.
        out.side_clearance = -object_half_extents_.y() - out.local.y();
        const bool inside_x = std::abs(out.local.x()) <= object_half_extents_.x();
        const bool inside_y = out.local.y() >= -object_half_extents_.y() &&
                              out.local.y() <= object_half_extents_.y();
        const bool inside_z = std::abs(out.local.z()) <= object_half_extents_.z();
        out.penetration_depth = (inside_x && inside_y && inside_z)
                                    ? std::max(0.0, -out.side_clearance)
                                    : 0.0;
        out.penetration_flag = out.penetration_depth > 1e-6 ? 1.0 : 0.0;
        out.keepout_violation = out.side_clearance < object_keepout_margin_ ? 1.0 : 0.0;
        return out;
    }

    static double finiteMin(double a, double b) {
        const bool af = std::isfinite(a);
        const bool bf = std::isfinite(b);
        if (af && bf) return std::min(a, b);
        if (af) return a;
        return b;
    }

    static double finiteMax(double a, double b) {
        const bool af = std::isfinite(a);
        const bool bf = std::isfinite(b);
        if (af && bf) return std::max(a, b);
        if (af) return a;
        return b;
    }

    void publishMetrics() {
        const ros::Time now = ros::Time::now();
        if (!object_pose_.valid || !fresh(object_pose_, now))
            return;

        const Pose leader_obj = objectGraspPose(leader_object_grasp_offset_, leader_object_grasp_quat_);
        const Pose follower_obj = objectGraspPose(follower_object_grasp_offset_, follower_object_grasp_quat_);

        const bool leader_ok = fresh(leader_ee_pose_, now);
        const bool follower_ok = fresh(follower_ee_pose_, now);
        const double leader_pos_error = leader_ok ? (leader_ee_pose_.p - leader_obj.p).norm() : -1.0;
        const double follower_pos_error = follower_ok ? (follower_ee_pose_.p - follower_obj.p).norm() : -1.0;
        const double leader_ori_error = leader_ok ? orientationError(leader_ee_pose_.q, leader_obj.q) : -1.0;
        const double follower_ori_error = follower_ok ? orientationError(follower_ee_pose_.q, follower_obj.q) : -1.0;

        std_msgs::Float32MultiArray gap;
        gap.data.push_back(static_cast<float>(leader_pos_error));
        gap.data.push_back(static_cast<float>(follower_pos_error));
        gap_pub_.publish(gap);

        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        objectRpy(roll, pitch, yaw);

        const Eigen::Vector3d nan_vec = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        const Eigen::Vector3d leader_ee_obj_local = leader_ok ? objectFramePoint(leader_ee_pose_.p) : nan_vec;
        const Eigen::Vector3d follower_ee_obj_local = follower_ok ? objectFramePoint(follower_ee_pose_.p) : nan_vec;
        const ClearanceSample follower_ee_clearance = followerSideClearance(follower_ee_pose_, now);
        const ClearanceSample follower_link7_clearance = followerSideClearance(follower_link7_pose_, now);
        const ClearanceSample follower_elbow_clearance = followerSideClearance(follower_elbow_pose_, now);
        double follower_arm_side_clearance_min = follower_ee_clearance.side_clearance;
        follower_arm_side_clearance_min = finiteMin(follower_arm_side_clearance_min, follower_link7_clearance.side_clearance);
        follower_arm_side_clearance_min = finiteMin(follower_arm_side_clearance_min, follower_elbow_clearance.side_clearance);
        double follower_arm_penetration_depth_max = follower_ee_clearance.penetration_depth;
        follower_arm_penetration_depth_max = finiteMax(follower_arm_penetration_depth_max, follower_link7_clearance.penetration_depth);
        follower_arm_penetration_depth_max = finiteMax(follower_arm_penetration_depth_max, follower_elbow_clearance.penetration_depth);
        const double follower_arm_penetration_flag =
            std::isfinite(follower_arm_penetration_depth_max) ? (follower_arm_penetration_depth_max > 1e-6 ? 1.0 : 0.0)
                                                              : std::numeric_limits<double>::quiet_NaN();
        const double follower_arm_keepout_violation =
            std::isfinite(follower_arm_side_clearance_min) ? (follower_arm_side_clearance_min < object_keepout_margin_ ? 1.0 : 0.0)
                                                           : std::numeric_limits<double>::quiet_NaN();

        std_msgs::Float32MultiArray err;
        err.data.reserve(44);
        err.data.push_back(static_cast<float>(leader_pos_error));                    // 0
        err.data.push_back(static_cast<float>(follower_pos_error));                  // 1
        err.data.push_back(static_cast<float>(leader_ori_error));                    // 2
        err.data.push_back(static_cast<float>(follower_ori_error));                  // 3
        err.data.push_back(static_cast<float>(object_pose_.p.z()));                  // 4 object_center_z
        err.data.push_back(static_cast<float>(leader_obj.p.z()));                    // 5 left/object leader grasp z
        err.data.push_back(static_cast<float>(follower_obj.p.z()));                  // 6 right/object follower grasp z
        err.data.push_back(static_cast<float>(leader_obj.p.z() - object_pose_.p.z()));   // 7
        err.data.push_back(static_cast<float>(follower_obj.p.z() - object_pose_.p.z())); // 8
        err.data.push_back(static_cast<float>(leader_obj.p.z() - follower_obj.p.z()));   // 9
        err.data.push_back(static_cast<float>(roll));                                // 10
        err.data.push_back(static_cast<float>(pitch));                               // 11
        err.data.push_back(static_cast<float>(yaw));                                 // 12
        err.data.push_back(static_cast<float>(leader_ok ? leader_ee_pose_.p.z() : -1.0));   // 13
        err.data.push_back(static_cast<float>(follower_ok ? follower_ee_pose_.p.z() : -1.0)); // 14
        err.data.push_back(static_cast<float>(leader_ok && follower_ok ? leader_ee_pose_.p.z() - follower_ee_pose_.p.z() : -1.0)); // 15
        err.data.push_back(static_cast<float>(object_pose_.p.x()));                  // 16 object_x
        err.data.push_back(static_cast<float>(object_pose_.p.y()));                  // 17 object_y
        const Eigen::Vector2d object_xy = object_pose_.p.head<2>();
        Eigen::Vector2d drift = Eigen::Vector2d::Zero();
        if (initial_object_xy_valid_)
            drift = object_xy - initial_object_xy_;
        err.data.push_back(static_cast<float>(drift.norm()));                        // 18 object_xy_drift
        err.data.push_back(static_cast<float>(std::atan2(drift.y(), drift.x())));    // 19 object_xy_drift_dir
        err.data.push_back(static_cast<float>(leader_ee_obj_local.x()));             // 20 leader_ee_obj_x
        err.data.push_back(static_cast<float>(leader_ee_obj_local.y()));             // 21 leader_ee_obj_y
        err.data.push_back(static_cast<float>(leader_ee_obj_local.z()));             // 22 leader_ee_obj_z
        err.data.push_back(static_cast<float>(follower_ee_obj_local.x()));           // 23 follower_ee_obj_x
        err.data.push_back(static_cast<float>(follower_ee_obj_local.y()));           // 24 follower_ee_obj_y
        err.data.push_back(static_cast<float>(follower_ee_obj_local.z()));           // 25 follower_ee_obj_z
        err.data.push_back(static_cast<float>(follower_ee_clearance.side_clearance)); // 26 follower_ee_side_clearance
        err.data.push_back(static_cast<float>(follower_ee_clearance.penetration_depth)); // 27 follower_ee_penetration_depth
        err.data.push_back(static_cast<float>(follower_ee_clearance.penetration_flag)); // 28 follower_ee_penetration_flag
        err.data.push_back(static_cast<float>(follower_ee_clearance.keepout_violation)); // 29 follower_ee_keepout_violation
        err.data.push_back(static_cast<float>(follower_link7_clearance.local.x()));   // 30 follower_link7_obj_x
        err.data.push_back(static_cast<float>(follower_link7_clearance.local.y()));   // 31 follower_link7_obj_y
        err.data.push_back(static_cast<float>(follower_link7_clearance.local.z()));   // 32 follower_link7_obj_z
        err.data.push_back(static_cast<float>(follower_link7_clearance.side_clearance)); // 33 follower_link7_side_clearance
        err.data.push_back(static_cast<float>(follower_link7_clearance.penetration_depth)); // 34 follower_link7_penetration_depth
        err.data.push_back(static_cast<float>(follower_elbow_clearance.local.x()));   // 35 follower_elbow_obj_x
        err.data.push_back(static_cast<float>(follower_elbow_clearance.local.y()));   // 36 follower_elbow_obj_y
        err.data.push_back(static_cast<float>(follower_elbow_clearance.local.z()));   // 37 follower_elbow_obj_z
        err.data.push_back(static_cast<float>(follower_elbow_clearance.side_clearance)); // 38 follower_elbow_side_clearance
        err.data.push_back(static_cast<float>(follower_elbow_clearance.penetration_depth)); // 39 follower_elbow_penetration_depth
        err.data.push_back(static_cast<float>(follower_arm_side_clearance_min));      // 40 follower_arm_side_clearance_min
        err.data.push_back(static_cast<float>(follower_arm_penetration_depth_max));   // 41 follower_arm_penetration_depth_max
        err.data.push_back(static_cast<float>(follower_arm_penetration_flag));        // 42 follower_arm_penetration_flag
        err.data.push_back(static_cast<float>(follower_arm_keepout_violation));       // 43 follower_arm_keepout_violation
        error_pub_.publish(err);

        ROS_DEBUG_THROTTLE(1.0,
            "[coop_grasp_tf_monitor] gap L/R=%.4f/%.4f ori=%.3f/%.3f z(center,L,R)=%.4f/%.4f/%.4f rpy=%.3f/%.3f/%.3f follower_clearance ee/link7/elbow=%.4f/%.4f/%.4f min=%.4f pen=%.4f",
            leader_pos_error, follower_pos_error, leader_ori_error, follower_ori_error,
            object_pose_.p.z(), leader_obj.p.z(), follower_obj.p.z(), roll, pitch, yaw,
            follower_ee_clearance.side_clearance,
            follower_link7_clearance.side_clearance,
            follower_elbow_clearance.side_clearance,
            follower_arm_side_clearance_min,
            follower_arm_penetration_depth_max);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber js_sub_;
    ros::Subscriber leader_sub_;
    ros::Subscriber follower_sub_;
    ros::Subscriber leader_link7_sub_;
    ros::Subscriber follower_link7_sub_;
    ros::Subscriber leader_elbow_sub_;
    ros::Subscriber follower_elbow_sub_;
    ros::Publisher gap_pub_;
    ros::Publisher error_pub_;

    std::string joint_states_topic_;
    std::string leader_ee_pose_topic_;
    std::string follower_ee_pose_topic_;
    std::string leader_link7_pose_topic_;
    std::string follower_link7_pose_topic_;
    std::string leader_elbow_pose_topic_;
    std::string follower_elbow_pose_topic_;
    std::string attach_gap_topic_;
    std::string grasp_tf_error_topic_;
    int object_qpos_offset_{40};
    double max_pose_age_{0.5};
    double rate_hz_{50.0};
    double object_keepout_margin_{0.03};

    Eigen::Vector3d object_half_extents_{0.267, 0.85, 0.025};
    Eigen::Vector3d leader_object_grasp_offset_{0.0, 0.85, 0.0};
    Eigen::Vector3d follower_object_grasp_offset_{0.0, -0.85, 0.0};
    Eigen::Quaterniond leader_object_grasp_quat_{Eigen::Quaterniond::Identity()};
    Eigen::Quaterniond follower_object_grasp_quat_{Eigen::Quaterniond::Identity()};

    Pose object_pose_;
    Pose leader_ee_pose_;
    Pose follower_ee_pose_;
    Pose leader_link7_pose_;
    Pose follower_link7_pose_;
    Pose leader_elbow_pose_;
    Pose follower_elbow_pose_;
    Eigen::Vector2d initial_object_xy_{Eigen::Vector2d::Zero()};
    bool initial_object_xy_valid_{false};
};

// -----------------------------------------------------------------------------
// 3/5 Object-centered pre-grasp and lift target generator
// -----------------------------------------------------------------------------
// A single object pose produces synchronized leader/follower references. Lift
// requests always switch the HQP back to external-target mode 200.
geometry_msgs::Transform toTransformMsg(const Eigen::Vector3d& p,
                                        const Eigen::Quaterniond& q_in) {
    const Eigen::Quaterniond q = normalizedQuaternion(q_in);

    geometry_msgs::Transform msg;
    msg.translation.x = p.x();
    msg.translation.y = p.y();
    msg.translation.z = p.z();
    msg.rotation.w = q.w();
    msg.rotation.x = q.x();
    msg.rotation.y = q.y();
    msg.rotation.z = q.z();
    return msg;
}

class CoopObjectGraspTargetNode {
public:
    CoopObjectGraspTargetNode() : pnh_("~") {
        pnh_.param<std::string>("joint_states_topic", joint_states_topic_,
                                "/coop_scene/mujoco_ros/mujoco_ros_interface/joint_states");
        pnh_.param<std::string>("ctrl_mode_topic", ctrl_mode_topic_, "/coop/ctrl_mode");
        pnh_.param<std::string>("leader_ee_target_topic", leader_target_topic_, "/leader/ee_target");
        pnh_.param<std::string>("follower_ee_target_topic", follower_target_topic_, "/follower/ee_target");
        pnh_.param<std::string>("leader_base_target_topic", leader_base_target_topic_, "/leader/base_target");
        pnh_.param<std::string>("follower_base_target_topic", follower_base_target_topic_, "/follower/base_target");
        pnh_.param<std::string>("leader_arm_posture_target_topic", leader_arm_posture_target_topic_, "/leader/arm_posture_target");
        pnh_.param<std::string>("follower_arm_posture_target_topic", follower_arm_posture_target_topic_, "/follower/arm_posture_target");
        pnh_.param<std::string>("attach_gap_topic", attach_gap_topic_, "/coop/attach_gap");
        pnh_.param<std::string>("leader_ee_pose_topic", leader_ee_pose_topic_, "/leader/ee_grasp_pose");
        pnh_.param<std::string>("follower_ee_pose_topic", follower_ee_pose_topic_, "/follower/ee_grasp_pose");
        pnh_.param<std::string>("cm_lift_delta_topic", cm_lift_delta_topic_, "/coop/cm_lift_delta");
        pnh_.param<int>("object_qpos_offset", object_qpos_offset_, 40);
        pnh_.param<int>("leader_base_qpos_offset", leader_base_qpos_offset_, 0);
        pnh_.param<int>("follower_base_qpos_offset", follower_base_qpos_offset_, 20);
        pnh_.param<int>("pregrasp_mode", pregrasp_mode_, 24);
        pnh_.param<int>("object_lift_mode", object_lift_mode_, 211);
        pnh_.param<int>("baseline_lift_mode", baseline_lift_mode_, -1);
        pnh_.param<bool>("debounce_mode_requests", debounce_mode_requests_, false);
        pnh_.param<int>("hqp_track_mode", hqp_track_mode_, 200);
        pnh_.param<double>("target_rate", rate_hz_, 20.0);
        pnh_.param<int>("target_publish_count", target_publish_count_, 3);
        pnh_.param<double>("target_publish_period", target_publish_period_, 0.2);
        pnh_.param<int>("pregrasp_z_stage_count", pregrasp_z_stage_count_, 0);
        pnh_.param<double>("object_lift_delta", object_lift_delta_, 0.03);
        pnh_.param<bool>("lift_level_orientation", lift_level_orientation_, true);
        pnh_.param<bool>("lift_use_canonical_grasp_offsets", lift_use_canonical_grasp_offsets_, true);
        pnh_.param<double>("max_target_step", max_target_step_, 0.25);
        pnh_.param<double>("max_base_target_step", max_base_target_step_, 0.35);
        pnh_.param<double>("target_error_gain", target_error_gain_, 0.0);
        pnh_.param<double>("max_target_error_correction", max_target_error_correction_, 0.25);
        pnh_.param<double>("skip_hqp_if_already_within_gap", skip_hqp_if_already_within_gap_, 0.0);
        pnh_.param<double>("aligned_gap_memory", aligned_gap_memory_, 2.0);
        pnh_.param<double>("max_pose_age", max_pose_age_, 0.5);
        pnh_.param<bool>("use_object_orientation", use_object_orientation_, false);
        pnh_.param<bool>("publish_base_targets", publish_base_targets_, true);
        pnh_.param<bool>("publish_base_targets_during_attached", publish_base_targets_during_attached_, false);
        pnh_.param<bool>("publish_arm_posture_target_during_lift", publish_arm_posture_target_during_lift_, false);
        readVectorParam(pnh_, "ee_grasp_offset_xyz", ee_grasp_offset_);
        readVectorParam(pnh_, "hqp_control_offset_xyz", hqp_control_offset_);
        readVectorParam(pnh_, "lift_arm_posture_target_q", lift_arm_posture_target_q_);
        readQuaternionParam(pnh_, "ee_grasp_offset_quat_wxyz", ee_grasp_quat_);
        readVectorParam(pnh_, "leader_object_grasp_offset_xyz", leader_object_grasp_offset_);
        readVectorParam(pnh_, "follower_object_grasp_offset_xyz", follower_object_grasp_offset_);
        readQuaternionParam(pnh_, "leader_object_grasp_offset_quat_wxyz", leader_object_grasp_quat_);
        readQuaternionParam(pnh_, "follower_object_grasp_offset_quat_wxyz", follower_object_grasp_quat_);
        readVectorParam(pnh_, "leader_target_world_offset_xyz", leader_target_world_offset_);
        readVectorParam(pnh_, "follower_target_world_offset_xyz", follower_target_world_offset_);
        readVectorParam(pnh_, "leader_base_target_world_offset_xyz", leader_base_target_world_offset_);
        readVectorParam(pnh_, "follower_base_target_world_offset_xyz", follower_base_target_world_offset_);

        js_sub_ = nh_.subscribe(joint_states_topic_, 5, &CoopObjectGraspTargetNode::jointStateCallback, this,
                               ros::TransportHints().tcpNoDelay(true));
        mode_sub_ = nh_.subscribe(ctrl_mode_topic_, 5, &CoopObjectGraspTargetNode::modeCallback, this);
        gap_sub_ = nh_.subscribe(attach_gap_topic_, 5, &CoopObjectGraspTargetNode::gapCallback, this);
        leader_ee_sub_ = nh_.subscribe(leader_ee_pose_topic_, 5, &CoopObjectGraspTargetNode::leaderEeCallback, this);
        follower_ee_sub_ = nh_.subscribe(follower_ee_pose_topic_, 5, &CoopObjectGraspTargetNode::followerEeCallback, this);
        if (!cm_lift_delta_topic_.empty())
            cm_lift_delta_sub_ = nh_.subscribe(cm_lift_delta_topic_, 1,
                                               &CoopObjectGraspTargetNode::cmLiftDeltaCallback, this);
        leader_target_pub_ = nh_.advertise<geometry_msgs::Transform>(leader_target_topic_, 5);
        follower_target_pub_ = nh_.advertise<geometry_msgs::Transform>(follower_target_topic_, 5);
        leader_base_target_pub_ = nh_.advertise<geometry_msgs::Transform>(leader_base_target_topic_, 5);
        follower_base_target_pub_ = nh_.advertise<geometry_msgs::Transform>(follower_base_target_topic_, 5);
        leader_arm_posture_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(leader_arm_posture_target_topic_, 5);
        follower_arm_posture_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(follower_arm_posture_target_topic_, 5);
        mode_pub_ = nh_.advertise<std_msgs::Int32>(ctrl_mode_topic_, 5);

        ROS_INFO("[coop_object_grasp_target] pregrasp_mode=%d object_lift_mode=%d baseline_lift_mode=%d lift_delta=%.3f hqp_track_mode=%d object_qpos_offset=%d max_step=%.3f base_step=%.3f z_stage=%d error_gain=%.2f skip_gap=%.3f object_orientation=%s lift_canonical_fallback=%s base_targets=%s attached_base_targets=%s lift_arm_posture=%s q=[%.3f %.3f %.3f %.3f %.3f %.3f %.3f] hqp_minus_grasp=[%.3f %.3f %.3f]",
                 pregrasp_mode_, object_lift_mode_, baseline_lift_mode_, object_lift_delta_,
                 hqp_track_mode_, object_qpos_offset_, max_target_step_,
                 max_base_target_step_, pregrasp_z_stage_count_, target_error_gain_,
                 skip_hqp_if_already_within_gap_,
                 use_object_orientation_ ? "true" : "false",
                 lift_use_canonical_grasp_offsets_ ? "true" : "false",
                 publish_base_targets_ ? "true" : "false",
                 publish_base_targets_during_attached_ ? "true" : "false",
                 publish_arm_posture_target_during_lift_ ? "true" : "false",
                 lift_arm_posture_target_q_(0), lift_arm_posture_target_q_(1),
                 lift_arm_posture_target_q_(2), lift_arm_posture_target_q_(3),
                 lift_arm_posture_target_q_(4), lift_arm_posture_target_q_(5),
                 lift_arm_posture_target_q_(6),
                 (hqp_control_offset_ - ee_grasp_offset_).x(),
                 (hqp_control_offset_ - ee_grasp_offset_).y(),
                 (hqp_control_offset_ - ee_grasp_offset_).z());
    }

    void update() {
        maybePublishTargets();
    }

    double rateHz() const {
        return rate_hz_;
    }

private:
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        if (static_cast<int>(msg->position.size()) < object_qpos_offset_ + 7)
            return;
        object_p_ = Eigen::Vector3d(msg->position[object_qpos_offset_ + 0],
                                    msg->position[object_qpos_offset_ + 1],
                                    msg->position[object_qpos_offset_ + 2]);
        object_q_ = normalizedQuaternion(Eigen::Quaterniond(
            msg->position[object_qpos_offset_ + 3], msg->position[object_qpos_offset_ + 4],
            msg->position[object_qpos_offset_ + 5], msg->position[object_qpos_offset_ + 6]));
        object_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        object_valid_ = true;
        readBasePose(msg, leader_base_qpos_offset_, leader_base_p_, leader_base_yaw_, leader_base_valid_);
        readBasePose(msg, follower_base_qpos_offset_, follower_base_p_, follower_base_yaw_, follower_base_valid_);
        maybeLatchInitialBaseOffsets();
    }

    void cmLiftDeltaCallback(const std_msgs::Float32ConstPtr& msg) {
        const double requested = static_cast<double>(msg->data);
        if (!std::isfinite(requested) || requested < -1.0 || requested > 1.0) {
            ROS_WARN("[coop_object_grasp_target] rejected CM lift delta %.3f", requested);
            return;
        }
        if (std::abs(requested - object_lift_delta_) <= 1e-9)
            return;
        object_lift_delta_ = requested;
        ROS_INFO("[coop_object_grasp_target] CM lift delta updated: %.3f", object_lift_delta_);
    }

    void modeCallback(const std_msgs::Int32ConstPtr& msg) {
        const int mode = msg->data;
        const bool lift_request = isObjectLiftMode(mode);
        if (mode != pregrasp_mode_ && !lift_request)
            return;
        if (!object_valid_) {
            ROS_WARN("[coop_object_grasp_target] mode %d requested but object pose is not available yet", mode);
            return;
        }
        // 수동 테스트에서는 같은 mode를 반복 입력해 목표를 다시 보낼 수 있다.
        if (debounce_mode_requests_ && mode == pending_request_mode_ &&
            pending_publish_count_ > 0) {
            ROS_INFO_THROTTLE(2.0, "[coop_object_grasp_target] mode %d already active; duplicate request ignored", mode);
            return;
        }
        pending_request_mode_ = mode;
        target_object_p_ = object_p_;
        target_object_q_ = object_q_;
        target_object_z_offset_ = lift_request ? object_lift_delta_ : 0.0;
        lift_baseline_valid_ = false;
        if (lift_request && liftReferenceFresh())
            lift_baseline_valid_ = captureLiftBaseline();
        else if (lift_request)
            ROS_WARN("[coop_object_grasp_target] mode %d lift requested without fresh object/leader/follower TF; using fallback target generation", mode);
        publish_base_targets_for_request_ =
            (mode == pregrasp_mode_) ? publish_base_targets_ :
                                        publish_base_targets_during_attached_;
        pending_publish_count_ = std::max(1, target_publish_count_);
        current_request_publish_count_ = pending_publish_count_;
        last_publish_time_ = ros::Time(0);
        ROS_INFO("[coop_object_grasp_target] mode %d requested; object-centered targets z_offset=%.3f base_targets=%s",
                 mode, target_object_z_offset_, publish_base_targets_for_request_ ? "true" : "false");
        if (lift_request)
            publishLiftArmPostureTarget();
    }

    bool isObjectLiftMode(int mode) const {
        return mode == object_lift_mode_ ||
               (baseline_lift_mode_ >= 0 && mode == baseline_lift_mode_);
    }

    bool freshStamp(const ros::Time& stamp, const ros::Time& now) const {
        return !stamp.isZero() && (now - stamp).toSec() <= max_pose_age_;
    }

    bool liftReferenceFresh() const {
        const ros::Time now = ros::Time::now();
        return object_valid_ && leader_ee_valid_ && follower_ee_valid_ &&
               freshStamp(object_stamp_, now) &&
               freshStamp(leader_ee_stamp_, now) &&
               freshStamp(follower_ee_stamp_, now);
    }

    bool captureLiftBaseline() {
        if (!leader_ee_valid_ || !follower_ee_valid_ || !object_valid_)
            return false;
        // 리프트 요청 순간의 object, leader EE, follower EE 3개 TF를
        // 방향까지 포함해 object frame에 고정한다.
        lift_object_p_ = object_p_;
        lift_object_q_ = object_q_;
        const Eigen::Matrix3d R0t = object_q_.toRotationMatrix().transpose();
        leader_grasp_in_obj_p_ = R0t * (leader_ee_p_ - object_p_);
        follower_grasp_in_obj_p_ = R0t * (follower_ee_p_ - object_p_);
        leader_grasp_in_obj_q_ = (object_q_.conjugate() * leader_ee_q_).normalized();
        follower_grasp_in_obj_q_ = (object_q_.conjugate() * follower_ee_q_).normalized();
        return true;
    }

    void gapCallback(const std_msgs::Float32MultiArrayConstPtr& msg) {
        if (msg->data.empty())
            return;
        double max_gap = 0.0;
        for (float v : msg->data)
            max_gap = std::max(max_gap, std::abs(static_cast<double>(v)));
        last_monitor_gap_ = max_gap;
        last_monitor_gap_stamp_ = ros::Time::now();
        if (skip_hqp_if_already_within_gap_ > 0.0 && max_gap <= skip_hqp_if_already_within_gap_)
            last_aligned_gap_stamp_ = last_monitor_gap_stamp_;
    }

    void setEePose(const geometry_msgs::TransformStamped::ConstPtr& msg,
                   Eigen::Vector3d& p,
                   Eigen::Quaterniond& q,
                   bool& valid) {
        p = Eigen::Vector3d(msg->transform.translation.x,
                            msg->transform.translation.y,
                            msg->transform.translation.z);
        q = normalizedQuaternion(Eigen::Quaterniond(
            msg->transform.rotation.w, msg->transform.rotation.x,
            msg->transform.rotation.y, msg->transform.rotation.z));
        valid = true;
    }

    void leaderEeCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
        setEePose(msg, leader_ee_p_, leader_ee_q_, leader_ee_valid_);
        leader_ee_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    }

    void followerEeCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
        setEePose(msg, follower_ee_p_, follower_ee_q_, follower_ee_valid_);
        follower_ee_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    }

    Eigen::Vector3d clampedTargetPosition(const Eigen::Vector3d& current,
                                          const Eigen::Vector3d& desired,
                                          bool has_current) const {
        if (!has_current || max_target_step_ <= 0.0)
            return desired;
        const Eigen::Vector3d delta = desired - current;
        const double dist = delta.norm();
        if (dist <= max_target_step_ || dist < 1e-9)
            return desired;
        return current + delta / dist * max_target_step_;
    }

    double yawFromQuaternion(const Eigen::Quaterniond& q) const {
        return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    }

    double wrapAngle(double angle) const {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    Eigen::Quaterniond yawQuaternion(double yaw) const {
        return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    }

    void readBasePose(const sensor_msgs::JointState::ConstPtr& msg,
                      int offset,
                      Eigen::Vector3d& p,
                      double& yaw,
                      bool& valid) {
        if (static_cast<int>(msg->position.size()) < offset + 7)
            return;
        p = Eigen::Vector3d(msg->position[offset + 0], msg->position[offset + 1], msg->position[offset + 2]);
        const Eigen::Quaterniond q = normalizedQuaternion(Eigen::Quaterniond(
            msg->position[offset + 3], msg->position[offset + 4],
            msg->position[offset + 5], msg->position[offset + 6]));
        yaw = yawFromQuaternion(q);
        valid = true;
    }

    void maybeLatchInitialBaseOffsets() {
        if (initial_base_offsets_valid_ || !object_valid_ || !leader_base_valid_ || !follower_base_valid_)
            return;
        const double object_yaw = yawFromQuaternion(object_q_);
        leader_base_object_offset_ = leader_base_p_ - object_p_;
        follower_base_object_offset_ = follower_base_p_ - object_p_;
        leader_base_yaw_offset_ = wrapAngle(leader_base_yaw_ - object_yaw);
        follower_base_yaw_offset_ = wrapAngle(follower_base_yaw_ - object_yaw);
        initial_base_offsets_valid_ = true;
        ROS_INFO("[coop_object_grasp_target] latched base-object offsets leader=[%.3f %.3f %.3f] follower=[%.3f %.3f %.3f]",
                 leader_base_object_offset_.x(), leader_base_object_offset_.y(), leader_base_object_offset_.z(),
                 follower_base_object_offset_.x(), follower_base_object_offset_.y(), follower_base_object_offset_.z());
    }

    Eigen::Vector3d clampedBaseTarget(const Eigen::Vector3d& current,
                                      const Eigen::Vector3d& desired,
                                      bool has_current) const {
        if (!has_current || max_base_target_step_ <= 0.0)
            return desired;
        Eigen::Vector3d delta = desired - current;
        delta.z() = 0.0;
        const double dist = delta.norm();
        if (dist <= max_base_target_step_ || dist < 1e-9)
            return desired;
        Eigen::Vector3d target = current + delta / dist * max_base_target_step_;
        target.z() = desired.z();
        return target;
    }

    Eigen::Vector3d correctedDesiredPosition(const Eigen::Vector3d& desired,
                                             const Eigen::Vector3d& current,
                                             bool has_current) const {
        if (!has_current || target_error_gain_ <= 0.0)
            return desired;
        Eigen::Vector3d correction = desired - current;
        const double norm = correction.norm();
        if (max_target_error_correction_ > 0.0 && norm > max_target_error_correction_)
            correction *= max_target_error_correction_ / norm;
        return desired + target_error_gain_ * correction;
    }

    Eigen::Quaterniond hqpQuatFromGraspQuat(const Eigen::Quaterniond& grasp_q) const {
        return normalizedQuaternion(grasp_q * ee_grasp_quat_.inverse());
    }

    Eigen::Vector3d hqpPointFromGraspPoint(const Eigen::Vector3d& grasp_p,
                                           const Eigen::Quaterniond& hqp_q) const {
        return grasp_p + hqp_q.toRotationMatrix() * (hqp_control_offset_ - ee_grasp_offset_);
    }

    void maybePublishTargets() {
        if (!object_valid_)
            return;
        if (pending_publish_count_ <= 0)
            return;
        const ros::Time now = ros::Time::now();
        if (pending_request_mode_ == pregrasp_mode_ &&
            skip_hqp_if_already_within_gap_ > 0.0 &&
            !last_aligned_gap_stamp_.isZero() &&
            (now - last_aligned_gap_stamp_).toSec() <= aligned_gap_memory_) {
            pending_publish_count_ = 0;
            ROS_INFO("[coop_object_grasp_target] pregrasp skipped: monitor gap %.4f <= %.4f within %.2fs",
                     last_monitor_gap_, skip_hqp_if_already_within_gap_, aligned_gap_memory_);
            return;
        }
        if (!last_publish_time_.isZero() &&
            (now - last_publish_time_).toSec() < target_publish_period_)
            return;

        Eigen::Vector3d target_object_p = target_object_p_;
        target_object_p.z() += target_object_z_offset_;
        const Eigen::Quaterniond target_object_q = target_object_q_;
        const Eigen::Matrix3d R = target_object_q.toRotationMatrix();
        const Eigen::Vector3d leader_desired_p =
            target_object_p + R * leader_object_grasp_offset_ + leader_target_world_offset_;
        const Eigen::Vector3d follower_desired_p =
            target_object_p + R * follower_object_grasp_offset_ + follower_target_world_offset_;
        const double leader_grasp_gap =
            leader_ee_valid_ ? (leader_ee_p_ - leader_desired_p).norm() : std::numeric_limits<double>::infinity();
        const double follower_grasp_gap =
            follower_ee_valid_ ? (follower_ee_p_ - follower_desired_p).norm() : std::numeric_limits<double>::infinity();
        const double max_grasp_gap = std::max(leader_grasp_gap, follower_grasp_gap);
        if (pending_request_mode_ == pregrasp_mode_ &&
            skip_hqp_if_already_within_gap_ > 0.0 &&
            max_grasp_gap <= skip_hqp_if_already_within_gap_) {
            pending_publish_count_ = 0;
            ROS_INFO("[coop_object_grasp_target] pregrasp already aligned: gap L/R=%.4f/%.4f <= %.4f; keeping current HQP mode",
                     leader_grasp_gap, follower_grasp_gap, skip_hqp_if_already_within_gap_);
            return;
        }
        const Eigen::Quaterniond leader_obj_q = (target_object_q * leader_object_grasp_quat_).normalized();
        const Eigen::Quaterniond follower_obj_q = (target_object_q * follower_object_grasp_quat_).normalized();
        const Eigen::Quaterniond leader_current_hqp_q = hqpQuatFromGraspQuat(leader_ee_q_);
        const Eigen::Quaterniond follower_current_hqp_q = hqpQuatFromGraspQuat(follower_ee_q_);
        const Eigen::Quaterniond leader_q =
            (use_object_orientation_ || !leader_ee_valid_) ? hqpQuatFromGraspQuat(leader_obj_q) : leader_current_hqp_q;
        const Eigen::Quaterniond follower_q =
            (use_object_orientation_ || !follower_ee_valid_) ? hqpQuatFromGraspQuat(follower_obj_q) : follower_current_hqp_q;
        const Eigen::Vector3d leader_desired_hqp_p = hqpPointFromGraspPoint(leader_desired_p, leader_q);
        const Eigen::Vector3d follower_desired_hqp_p = hqpPointFromGraspPoint(follower_desired_p, follower_q);
        const Eigen::Vector3d leader_current_hqp_p =
            hqpPointFromGraspPoint(leader_ee_p_, leader_current_hqp_q);
        const Eigen::Vector3d follower_current_hqp_p =
            hqpPointFromGraspPoint(follower_ee_p_, follower_current_hqp_q);
        Eigen::Vector3d leader_p;
        Eigen::Vector3d follower_p;
        Eigen::Quaterniond leader_cmd_q = leader_q;
        Eigen::Quaterniond follower_cmd_q = follower_q;
        const int published_count = std::max(0, current_request_publish_count_ - pending_publish_count_);
        const bool pregrasp_z_stage =
            (pending_request_mode_ == pregrasp_mode_ &&
             pregrasp_z_stage_count_ > 0 &&
             published_count < pregrasp_z_stage_count_);

        Eigen::Vector3d base_object_ref_p = target_object_p;
        Eigen::Vector3d leader_base_object_ref_p = base_object_ref_p;
        Eigen::Vector3d follower_base_object_ref_p = base_object_ref_p;
        Eigen::Quaterniond base_object_ref_q = target_object_q;

        if (isObjectLiftMode(pending_request_mode_) &&
            (lift_baseline_valid_ || lift_use_canonical_grasp_offsets_)) {
            // 하나의 물체 기준 자세에서 양쪽 EE 목표를 함께 생성해
            // 물체 자세 안정성을 우선한다.
            const int total = std::max(1, current_request_publish_count_);
            const int done = std::max(0, total - pending_publish_count_);
            const double ramp = static_cast<double>(done + 1) / static_cast<double>(total);
            Eigen::Quaterniond Oref_q = lift_object_q_;
            if (lift_level_orientation_) {
                const double base_yaw = yawFromQuaternion(lift_object_q_);
                Oref_q = yawQuaternion(base_yaw);  // roll/pitch를 0으로 맞춰 수평 유지
            }
            Eigen::Vector3d Oref_p = lift_object_p_;
            Oref_p.z() += object_lift_delta_ * ramp;
            Eigen::Vector3d base_object_ref_override_p = Oref_p;
            Eigen::Vector3d leader_base_object_ref_override_p = base_object_ref_override_p;
            Eigen::Vector3d follower_base_object_ref_override_p = base_object_ref_override_p;
            Eigen::Quaterniond base_object_ref_override_q = Oref_q;
            const bool baseline_lift_request =
                (baseline_lift_mode_ >= 0 && pending_request_mode_ == baseline_lift_mode_);
            if (lift_baseline_valid_) {
                // 요청 순간의 object-leader-follower 3개 TF 관계를 보존해 한 번에 들어올린다.
                const Eigen::Quaterniond object_ref_q =
                    baseline_lift_request ? lift_object_q_ : Oref_q;
                const Eigen::Matrix3d Rref = object_ref_q.toRotationMatrix();
                const Eigen::Quaterniond leader_grasp_q =
                    (object_ref_q * leader_grasp_in_obj_q_).normalized();
                const Eigen::Quaterniond follower_grasp_q =
                    (object_ref_q * follower_grasp_in_obj_q_).normalized();
                leader_cmd_q = hqpQuatFromGraspQuat(leader_grasp_q);
                follower_cmd_q = hqpQuatFromGraspQuat(follower_grasp_q);
                const Eigen::Vector3d leader_grasp_p =
                    Oref_p + Rref * leader_grasp_in_obj_p_;
                const Eigen::Vector3d follower_grasp_p =
                    Oref_p + Rref * follower_grasp_in_obj_p_;
                leader_p = hqpPointFromGraspPoint(leader_grasp_p, leader_cmd_q);
                follower_p = hqpPointFromGraspPoint(follower_grasp_p, follower_cmd_q);
                base_object_ref_override_q = object_ref_q;
            } else {
                const Eigen::Matrix3d Rref = Oref_q.toRotationMatrix();
                const Eigen::Quaterniond leader_grasp_q =
                    (Oref_q * leader_object_grasp_quat_).normalized();
                const Eigen::Quaterniond follower_grasp_q =
                    (Oref_q * follower_object_grasp_quat_).normalized();
                leader_cmd_q = hqpQuatFromGraspQuat(leader_grasp_q);
                follower_cmd_q = hqpQuatFromGraspQuat(follower_grasp_q);
                const Eigen::Vector3d leader_grasp_p =
                    Oref_p + Rref * leader_object_grasp_offset_ + leader_target_world_offset_;
                const Eigen::Vector3d follower_grasp_p =
                    Oref_p + Rref * follower_object_grasp_offset_ + follower_target_world_offset_;
                leader_p = hqpPointFromGraspPoint(leader_grasp_p, leader_cmd_q);
                follower_p = hqpPointFromGraspPoint(follower_grasp_p, follower_cmd_q);
            }
            base_object_ref_p = base_object_ref_override_p;
            leader_base_object_ref_p = leader_base_object_ref_override_p;
            follower_base_object_ref_p = follower_base_object_ref_override_p;
            base_object_ref_q = base_object_ref_override_q;
        } else if (pregrasp_z_stage) {
            // 접근 단계에서는 z축을 먼저 맞춘 뒤 전체 목표로 이동한다.
            Eigen::Vector3d leader_z_target = leader_current_hqp_p;
            Eigen::Vector3d follower_z_target = follower_current_hqp_p;
            leader_z_target.z() = leader_desired_hqp_p.z();
            follower_z_target.z() = follower_desired_hqp_p.z();
            leader_p = clampedTargetPosition(leader_current_hqp_p, leader_z_target, leader_ee_valid_);
            follower_p = clampedTargetPosition(follower_current_hqp_p, follower_z_target, follower_ee_valid_);
        } else {
            const Eigen::Vector3d leader_corrected_p =
                correctedDesiredPosition(leader_desired_hqp_p, leader_current_hqp_p, leader_ee_valid_);
            const Eigen::Vector3d follower_corrected_p =
                correctedDesiredPosition(follower_desired_hqp_p, follower_current_hqp_p, follower_ee_valid_);
            leader_p = clampedTargetPosition(leader_current_hqp_p, leader_corrected_p, leader_ee_valid_);
            follower_p = clampedTargetPosition(follower_current_hqp_p, follower_corrected_p, follower_ee_valid_);
        }

        leader_target_pub_.publish(toTransformMsg(leader_p, leader_cmd_q));
        follower_target_pub_.publish(toTransformMsg(follower_p, follower_cmd_q));

        if (publish_base_targets_for_request_ && initial_base_offsets_valid_) {
            const double object_yaw = yawFromQuaternion(base_object_ref_q);
            Eigen::Vector3d leader_base_des =
                leader_base_object_ref_p + leader_base_object_offset_ + leader_base_target_world_offset_;
            Eigen::Vector3d follower_base_des =
                follower_base_object_ref_p + follower_base_object_offset_ + follower_base_target_world_offset_;
            leader_base_des = clampedBaseTarget(leader_base_p_, leader_base_des, leader_base_valid_);
            follower_base_des = clampedBaseTarget(follower_base_p_, follower_base_des, follower_base_valid_);
            const double leader_yaw_des = object_yaw + leader_base_yaw_offset_;
            const double follower_yaw_des = object_yaw + follower_base_yaw_offset_;
            leader_base_target_pub_.publish(toTransformMsg(leader_base_des, yawQuaternion(leader_yaw_des)));
            follower_base_target_pub_.publish(toTransformMsg(follower_base_des, yawQuaternion(follower_yaw_des)));
        }

        if (isObjectLiftMode(pending_request_mode_))
            publishLiftArmPostureTarget();

        std_msgs::Int32 mode;
        mode.data = hqp_track_mode_;
        mode_pub_.publish(mode);

        --pending_publish_count_;
        last_publish_time_ = now;
        ROS_INFO("[coop_object_grasp_target] target publish mode=%d remain=%d stage=%s object=[%.3f %.3f %.3f] leader=[%.3f %.3f %.3f] follower=[%.3f %.3f %.3f]",
                 pending_request_mode_, pending_publish_count_,
                 pregrasp_z_stage ? "pregrasp_z" : "full",
                 target_object_p.x(), target_object_p.y(), target_object_p.z(),
                 leader_p.x(), leader_p.y(), leader_p.z(),
                 follower_p.x(), follower_p.y(), follower_p.z());
    }

    void publishLiftArmPostureTarget() {
        if (!publish_arm_posture_target_during_lift_)
            return;
        std_msgs::Float64MultiArray msg;
        msg.data.resize(7);
        for (int i = 0; i < 7; ++i)
            msg.data[i] = lift_arm_posture_target_q_(i);
        leader_arm_posture_pub_.publish(msg);
        follower_arm_posture_pub_.publish(msg);
        ROS_INFO_THROTTLE(1.0,
                          "[coop_object_grasp_target] lift arm posture target published q=[%.3f %.3f %.3f %.3f %.3f %.3f %.3f]",
                          lift_arm_posture_target_q_(0), lift_arm_posture_target_q_(1),
                          lift_arm_posture_target_q_(2), lift_arm_posture_target_q_(3),
                          lift_arm_posture_target_q_(4), lift_arm_posture_target_q_(5),
                          lift_arm_posture_target_q_(6));
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber js_sub_;
    ros::Subscriber mode_sub_;
    ros::Subscriber gap_sub_;
    ros::Subscriber leader_ee_sub_;
    ros::Subscriber follower_ee_sub_;
    ros::Subscriber cm_lift_delta_sub_;
    ros::Publisher leader_target_pub_;
    ros::Publisher follower_target_pub_;
    ros::Publisher leader_base_target_pub_;
    ros::Publisher follower_base_target_pub_;
    ros::Publisher leader_arm_posture_pub_;
    ros::Publisher follower_arm_posture_pub_;
    ros::Publisher mode_pub_;

    std::string joint_states_topic_;
    std::string ctrl_mode_topic_;
    std::string leader_target_topic_;
    std::string follower_target_topic_;
    std::string leader_base_target_topic_;
    std::string follower_base_target_topic_;
    std::string leader_arm_posture_target_topic_;
    std::string follower_arm_posture_target_topic_;
    std::string attach_gap_topic_;
    std::string leader_ee_pose_topic_;
    std::string follower_ee_pose_topic_;
    std::string cm_lift_delta_topic_;
    int object_qpos_offset_{40};
    int leader_base_qpos_offset_{0};
    int follower_base_qpos_offset_{20};
    int pregrasp_mode_{24};
    int object_lift_mode_{211};
    int baseline_lift_mode_{-1};
    int hqp_track_mode_{200};
    double rate_hz_{20.0};
    int target_publish_count_{3};
    int current_request_publish_count_{0};
    double target_publish_period_{0.2};
    int pregrasp_z_stage_count_{0};
    double object_lift_delta_{0.03};
    double max_target_step_{0.25};
    double max_base_target_step_{0.35};
    double target_error_gain_{0.0};
    double max_target_error_correction_{0.25};
    double skip_hqp_if_already_within_gap_{0.0};
    double aligned_gap_memory_{2.0};
    double max_pose_age_{0.5};
    bool use_object_orientation_{false};
    bool publish_base_targets_{true};
    bool publish_base_targets_during_attached_{false};
    bool publish_arm_posture_target_during_lift_{false};
    bool lift_baseline_valid_{false};
    int pending_publish_count_{0};
    int pending_request_mode_{0};
    bool debounce_mode_requests_{false};
    double target_object_z_offset_{0.0};
    bool publish_base_targets_for_request_{true};
    ros::Time last_publish_time_;
    ros::Time last_monitor_gap_stamp_;
    ros::Time last_aligned_gap_stamp_;
    double last_monitor_gap_{std::numeric_limits<double>::infinity()};

    Eigen::Vector3d leader_object_grasp_offset_{0.0, 0.85, 0.0};
    Eigen::Vector3d follower_object_grasp_offset_{0.0, -0.85, 0.0};
    Eigen::Vector3d ee_grasp_offset_{0.0, 0.0, 0.1654};
    Eigen::Vector3d hqp_control_offset_{0.0, 0.0, 0.2054};
    Eigen::Quaterniond leader_object_grasp_quat_{Eigen::Quaterniond::Identity()};
    Eigen::Quaterniond follower_object_grasp_quat_{Eigen::Quaterniond::Identity()};
    Eigen::Quaterniond ee_grasp_quat_{0.0, 1.0, 0.0, 0.0};
    Eigen::Vector3d leader_target_world_offset_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d follower_target_world_offset_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d leader_base_target_world_offset_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d follower_base_target_world_offset_{Eigen::Vector3d::Zero()};
    Eigen::Matrix<double, 7, 1> lift_arm_posture_target_q_{
        (Eigen::Matrix<double, 7, 1>() << 0.0, 0.0, 0.0, -M_PI / 2.0, 0.0, M_PI / 2.0, -M_PI / 4.0).finished()
    };

    Eigen::Vector3d object_p_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond object_q_{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d target_object_p_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond target_object_q_{Eigen::Quaterniond::Identity()};
    ros::Time object_stamp_;
    bool object_valid_{false};
    Eigen::Vector3d leader_base_p_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d follower_base_p_{Eigen::Vector3d::Zero()};
    double leader_base_yaw_{0.0};
    double follower_base_yaw_{0.0};
    bool leader_base_valid_{false};
    bool follower_base_valid_{false};
    bool initial_base_offsets_valid_{false};
    Eigen::Vector3d leader_base_object_offset_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d follower_base_object_offset_{Eigen::Vector3d::Zero()};
    double leader_base_yaw_offset_{0.0};
    double follower_base_yaw_offset_{0.0};
    Eigen::Vector3d leader_ee_p_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d follower_ee_p_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond leader_ee_q_{Eigen::Quaterniond::Identity()};
    Eigen::Quaterniond follower_ee_q_{Eigen::Quaterniond::Identity()};
    ros::Time leader_ee_stamp_;
    ros::Time follower_ee_stamp_;
    bool leader_ee_valid_{false};
    bool follower_ee_valid_{false};
    // 리프트 요청 시 고정되는 object 기준 3-TF 상태.
    bool lift_level_orientation_{true};
    bool lift_use_canonical_grasp_offsets_{true};
    Eigen::Vector3d lift_object_p_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond lift_object_q_{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d leader_grasp_in_obj_p_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d follower_grasp_in_obj_p_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond leader_grasp_in_obj_q_{Eigen::Quaterniond::Identity()};
    Eigen::Quaterniond follower_grasp_in_obj_q_{Eigen::Quaterniond::Identity()};
};

// -----------------------------------------------------------------------------
// 4/5 Leader/follower command merger
// -----------------------------------------------------------------------------
// C++ command arbitration layer for the dual cooperative transport scene.
// Leader and follower controllers each publish one single-robot 13-actuator
// JointSet. This node is the only runtime publisher to MuJoCo's final 26-actuator
// joint_set topic, preventing command overwrite between the two controllers.
class CoopJointSetMerger {
public:
    CoopJointSetMerger()
        : pnh_("~"),
          leader_(command_size_each_, 0.0),
          follower_(command_size_each_, 0.0) {
        pnh_.param("merger_rate", rate_hz_, 500.0);
        pnh_.param("command_size_each", command_size_each_, 13);
        pnh_.param("output_size", output_size_, 26);
        pnh_.param("timeout", timeout_, 0.25);
        pnh_.param("start_delay", start_delay_, 0.0);
        pnh_.param("mode", mode_, 1);
        pnh_.param("zero_on_timeout", zero_on_timeout_, true);
        pnh_.param("publish_on_command", publish_on_command_, false);
        pnh_.param("enable_debug_log", enable_debug_log_, false);
        pnh_.param("enable_debug_topic", enable_debug_topic_, true);
        pnh_.param<std::string>("leader_command_topic", leader_topic_, "/leader/joint_command");
        pnh_.param<std::string>("follower_command_topic", follower_topic_, "/follower/joint_command");
        pnh_.param<std::string>("output_topic", output_topic_,
                                "/coop_scene/mujoco_ros/mujoco_ros_interface/joint_set");
        pnh_.param<std::string>("sim_time_topic", sim_time_topic_,
                                "/coop_scene/mujoco_ros/mujoco_ros_interface/sim_time");

        leader_.assign(command_size_each_, 0.0);
        follower_.assign(command_size_each_, 0.0);

        leader_sub_ = nh_.subscribe(leader_topic_, 5, &CoopJointSetMerger::leaderCallback, this,
                                    ros::TransportHints().tcpNoDelay(true));
        follower_sub_ = nh_.subscribe(follower_topic_, 5, &CoopJointSetMerger::followerCallback, this,
                                      ros::TransportHints().tcpNoDelay(true));
        sim_time_sub_ = nh_.subscribe(sim_time_topic_, 1, &CoopJointSetMerger::simTimeCallback, this);
        output_pub_ = nh_.advertise<mujoco_ros_msgs::JointSet>(output_topic_, 1);
        debug_pub_ = pnh_.advertise<std_msgs::Float32MultiArray>("debug", 10);
        start_wall_time_ = ros::Time::now();

        ROS_INFO("[coop_joint_set_merger] leader=%s follower=%s output=%s publish_on_command=%s start_delay=%.2f",
                 leader_topic_.c_str(), follower_topic_.c_str(), output_topic_.c_str(),
                 publish_on_command_ ? "true" : "false", start_delay_);
    }

    void update() {
        publishMerged();
    }

    double rateHz() const {
        return publish_on_command_ ? std::min(rate_hz_, 100.0) : rate_hz_;
    }

private:
    static std::vector<double> fitCommand(const std::vector<double>& in, int size) {
        std::vector<double> out(size, 0.0);
        for (int i = 0; i < size && i < static_cast<int>(in.size()); ++i)
            out[i] = in[i];
        return out;
    }

    static bool isFiniteCommand(const std::vector<double>& command) {
        return std::all_of(
            command.begin(), command.end(),
            [](double value) { return std::isfinite(value); });
    }

    std::vector<double> commandOrZero(const std::vector<double>& command,
                                      const ros::Time& stamp,
                                      bool* fresh) const {
        if (stamp.isZero()) {
            *fresh = false;
            return std::vector<double>(command_size_each_, 0.0);
        }
        const double age = (ros::Time::now() - stamp).toSec();
        *fresh = age <= timeout_;
        if (!*fresh && zero_on_timeout_)
            return std::vector<double>(command_size_each_, 0.0);
        return command;
    }

    void simTimeCallback(const std_msgs::Float32ConstPtr& msg) {
        sim_time_ = msg->data;
    }

    void leaderCallback(const mujoco_ros_msgs::JointSetConstPtr& msg) {
        if (!isFiniteCommand(msg->torque)) {
            leader_.assign(command_size_each_, 0.0);
            leader_stamp_ = ros::Time();
            ROS_ERROR_THROTTLE(1.0, "[coop_joint_set_merger] rejected non-finite leader command");
            return;
        }
        leader_ = fitCommand(msg->torque, command_size_each_);
        leader_stamp_ = ros::Time::now();
        if (publish_on_command_)
            publishMerged();
    }

    void followerCallback(const mujoco_ros_msgs::JointSetConstPtr& msg) {
        if (!isFiniteCommand(msg->torque)) {
            follower_.assign(command_size_each_, 0.0);
            follower_stamp_ = ros::Time();
            ROS_ERROR_THROTTLE(1.0, "[coop_joint_set_merger] rejected non-finite follower command");
            return;
        }
        follower_ = fitCommand(msg->torque, command_size_each_);
        follower_stamp_ = ros::Time::now();
    }

    void publishDebug(bool leader_fresh, bool follower_fresh, double max_abs) {
        if (!enable_debug_topic_)
            return;
        std_msgs::Float32MultiArray msg;
        std_msgs::MultiArrayDimension dim;
        dim.label = "leader_fresh,follower_fresh,max_abs_command,publish_count";
        dim.size = 4;
        dim.stride = 4;
        msg.layout.dim.push_back(dim);
        msg.data.push_back(leader_fresh ? 1.0f : 0.0f);
        msg.data.push_back(follower_fresh ? 1.0f : 0.0f);
        msg.data.push_back(static_cast<float>(max_abs));
        msg.data.push_back(static_cast<float>(publish_count_));
        debug_pub_.publish(msg);
    }

    void publishMerged() {
        if (start_delay_ > 0.0 && !start_wall_time_.isZero()) {
            const double elapsed = (ros::Time::now() - start_wall_time_).toSec();
            if (elapsed < start_delay_)
                return;
        }

        bool leader_fresh = false;
        bool follower_fresh = false;
        const std::vector<double> leader = commandOrZero(leader_, leader_stamp_, &leader_fresh);
        const std::vector<double> follower = commandOrZero(follower_, follower_stamp_, &follower_fresh);

        std::vector<double> torque;
        torque.reserve(output_size_);
        torque.insert(torque.end(), leader.begin(), leader.end());
        torque.insert(torque.end(), follower.begin(), follower.end());
        torque = fitCommand(torque, output_size_);

        mujoco_ros_msgs::JointSet out;
        out.header.stamp = ros::Time::now();
        out.time = sim_time_;
        out.MODE = mode_;
        out.position.assign(output_size_, 0.0);
        out.torque = torque;
        output_pub_.publish(out);
        ++publish_count_;

        double max_abs = 0.0;
        for (double v : torque)
            max_abs = std::max(max_abs, std::abs(v));
        publishDebug(leader_fresh, follower_fresh, max_abs);
        if (enable_debug_log_) {
            ROS_INFO_THROTTLE(1.0, "[coop_joint_set_merger] max=%.3f leader_fresh=%s follower_fresh=%s",
                              max_abs, leader_fresh ? "true" : "false",
                              follower_fresh ? "true" : "false");
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber leader_sub_;
    ros::Subscriber follower_sub_;
    ros::Subscriber sim_time_sub_;
    ros::Publisher output_pub_;
    ros::Publisher debug_pub_;

    int command_size_each_ = 13;
    int output_size_ = 26;
    int mode_ = 1;
    double rate_hz_ = 100.0;
    double timeout_ = 0.25;
    double start_delay_ = 0.0;
    double sim_time_ = 0.0;
    bool zero_on_timeout_ = true;
    bool publish_on_command_ = false;
    bool enable_debug_log_ = false;
    bool enable_debug_topic_ = true;
    std::string leader_topic_;
    std::string follower_topic_;
    std::string output_topic_;
    std::string sim_time_topic_;

    std::vector<double> leader_;
    std::vector<double> follower_;
    ros::Time leader_stamp_;
    ros::Time follower_stamp_;
    ros::Time start_wall_time_;
    int publish_count_ = 0;
};

// -----------------------------------------------------------------------------
// 5/5 CM seed, keyboard, and lift-readiness coordinator
// -----------------------------------------------------------------------------

const std::array<double, 7> kArmLower = {
    -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
const std::array<double, 7> kArmUpper = {
    2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};

struct BasePair {
    std::array<double, 3> leader;
    std::array<double, 3> follower;
};

std::vector<double> requireVector7(const YAML::Node& node, const std::string& name) {
    if (!node || !node.IsSequence() || node.size() != 7)
        throw std::runtime_error(name + " must be a 7-element list");

    std::vector<double> values;
    values.reserve(7);
    for (std::size_t i = 0; i < node.size(); ++i)
        values.push_back(node[i].as<double>());
    return values;
}

std::array<double, 3> requirePose3(const YAML::Node& node, const std::string& name) {
    if (!node || !node.IsSequence() || node.size() != 3)
        throw std::runtime_error(name + " must be a 3-element [x, y, yaw] list");

    return {{node[0].as<double>(), node[1].as<double>(), node[2].as<double>()}};
}

std::string formatVector(const std::vector<double>& values) {
    std::ostringstream out;
    out << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i > 0)
            out << ", ";
        out << std::fixed << std::setprecision(4) << values[i];
    }
    out << "]";
    return out.str();
}

std::string formatBases(const BasePair& bases) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "L[" << bases.leader[0] << " " << bases.leader[1] << " " << bases.leader[2]
        << "] F[" << bases.follower[0] << " " << bases.follower[1] << " " << bases.follower[2]
        << "]";
    return out.str();
}

double degToRad(double deg) {
    return deg * M_PI / 180.0;
}

geometry_msgs::Transform toTransformMsg(const std::array<double, 3>& xyyaw) {
    geometry_msgs::Transform msg;
    const double yaw = xyyaw[2];
    msg.translation.x = xyyaw[0];
    msg.translation.y = xyyaw[1];
    msg.translation.z = 0.0;
    msg.rotation.w = std::cos(0.5 * yaw);
    msg.rotation.x = 0.0;
    msg.rotation.y = 0.0;
    msg.rotation.z = std::sin(0.5 * yaw);
    return msg;
}

const char* kKeyboardHelp =
    "\nDual cooperative transport - keyboard mode\n"
    "------------------------------------------\n"
    "Home-lift welded sequence: c -> t -> 0\n"
    "Holding-CM sequence: c -> y -> 0\n"
    "\n"
    "h : direct HQP home mode 1 (manual)\n"
    "p : manual re-approach/recalibration, mode 24\n"
    "c : attach weld\n"
    "t : baseline lift to home posture through mode 200, request 211\n"
    "y : holding-CM lift, mode 238\n"
    "0 : final welded transport, mode 235\n"
    "? : help\n"
    "Ctrl-C : exit\n";

class TerminalRawMode {
public:
    ~TerminalRawMode() {
        restore();
    }

    bool enable() {
        if (enabled_)
            return valid_;

        valid_ = isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &old_) == 0;
        if (valid_) {
            termios raw = old_;
            raw.c_lflag &= ~(ICANON | ECHO);
            raw.c_cc[VMIN] = 0;
            raw.c_cc[VTIME] = 0;
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        }
        enabled_ = true;
        return valid_;
    }

    bool valid() const {
        return valid_;
    }

private:
    void restore() {
        if (valid_)
            tcsetattr(STDIN_FILENO, TCSANOW, &old_);
        valid_ = false;
    }

    bool enabled_ = false;
    bool valid_ = false;
    termios old_;
};

bool readKey(char* key) {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    const int ret = select(STDIN_FILENO + 1, &set, nullptr, nullptr, &timeout);
    if (ret <= 0)
        return false;
    return read(STDIN_FILENO, key, 1) == 1;
}

void printKeyboardHelp() {
    const ssize_t ret = ::write(
        STDOUT_FILENO,
        kKeyboardHelp,
        std::char_traits<char>::length(kKeyboardHelp));
    (void)ret;
}

class CoopCmRuntimeNode {
public:
    CoopCmRuntimeNode() : nh_(), pnh_("~") {
        loadParams();
        loadSeed();

        q_cm_leader_ = applyJoint13Offset(q_cm_raw_, leader_joint1_offset_deg_, leader_joint3_offset_deg_);
        q_cm_follower_ = applyJoint13Offset(q_cm_raw_, follower_joint1_offset_deg_, follower_joint3_offset_deg_);

        leader_arm_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(leader_arm_topic_, 2);
        follower_arm_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(follower_arm_topic_, 2);
        leader_base_pub_ = nh_.advertise<geometry_msgs::Transform>(leader_base_topic_, 2);
        follower_base_pub_ = nh_.advertise<geometry_msgs::Transform>(follower_base_topic_, 2);
        lift_delta_pub_ = nh_.advertise<std_msgs::Float32>(lift_delta_topic_, 1, true);
        transport_ready_pub_ = nh_.advertise<std_msgs::Bool>(transport_ready_topic_, 1, true);
        lift_ready_pub_ = nh_.advertise<std_msgs::Bool>(lift_ready_topic_, 1, true);
        lift_status_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(lift_status_topic_, 5);
        mode_pub_ = nh_.advertise<std_msgs::Int32>(ctrl_mode_topic_, 1);

        mode_sub_ = nh_.subscribe(ctrl_mode_topic_, 10, &CoopCmRuntimeNode::modeCallback, this);
        ready_sub_ = nh_.subscribe(seed_ready_topic_, 10, &CoopCmRuntimeNode::readyCallback, this);
        metrics_sub_ = nh_.subscribe(metrics_topic_, 10, &CoopCmRuntimeNode::metricsCallback, this);
        joint_states_sub_ = nh_.subscribe(joint_states_topic_, 10, &CoopCmRuntimeNode::jointStatesCallback, this,
                                          ros::TransportHints().tcpNoDelay(true));
        lift_delta_sub_ = nh_.subscribe(lift_delta_topic_, 1, &CoopCmRuntimeNode::liftDeltaCallback, this);

        target_timer_ = nh_.createTimer(ros::Duration(publish_period_), &CoopCmRuntimeNode::targetTimerCallback, this);
        lift_timer_ = nh_.createTimer(
            ros::Duration(1.0 / std::max(1.0, lift_ready_rate_hz_)),
            &CoopCmRuntimeNode::liftTimerCallback,
            this);
        if (enable_keyboard_) {
            if (terminal_.enable()) {
                keyboard_timer_ = nh_.createTimer(
                    ros::Duration(1.0 / std::max(1.0, keyboard_rate_hz_)),
                    &CoopCmRuntimeNode::keyboardTimerCallback,
                    this);
                printKeyboardHelp();
                ROS_INFO_STREAM("[coop_cm_runtime] keyboard mode publishes ctrl_mode to " << ctrl_mode_topic_);
            } else {
                ROS_WARN("[coop_cm_runtime] stdin is not a TTY; use rostopic pub for /coop/ctrl_mode");
            }
        }

        publishLiftDelta();
        publishTransportReady(false);
        publishLiftReady(false);

        ROS_INFO_STREAM(
            "[coop_cm_runtime] seed=" << cm_seed_file_
            << " candidate_group=" << candidate_group_
            << " candidate_index=" << candidate_index_
            << " lift_delta=" << std::fixed << std::setprecision(3) << lift_delta_
            << " holding_q_raw=" << formatVector(q_cm_raw_)
            << " holding_q_leader=" << formatVector(q_cm_leader_)
            << " holding_q_follower=" << formatVector(q_cm_follower_)
            << " holding_base=" << formatBases(cm_bases_)
            << " initial_base=" << formatBases(initial_bases_)
            << " arm_posture_on_lift=" << (publish_arm_posture_on_lift_ ? "true" : "false")
            << " arm_posture_after_ready=" << (publish_arm_posture_after_ready_ ? "true" : "false")
            << " base_on_lift=" << (publish_base_on_lift_ ? "true" : "false")
            << " base_after_ready=" << (publish_base_after_ready_ ? "true" : "false")
            << " lift_ready_topic=" << seed_ready_topic_
            << " transport_ready_topic=" << transport_ready_topic_);

        if (std::abs(leader_joint1_offset_deg_) > 1e-9 ||
            std::abs(leader_joint3_offset_deg_) > 1e-9 ||
            std::abs(follower_joint1_offset_deg_) > 1e-9 ||
            std::abs(follower_joint3_offset_deg_) > 1e-9) {
            ROS_WARN_STREAM(
                "[coop_cm_runtime] experimental joint13 posture bias enabled: raw_holding_q="
                << formatVector(q_cm_raw_)
                << " leader_offsets=(" << leader_joint1_offset_deg_ << ", "
                << leader_joint3_offset_deg_ << ")deg leader_q=" << formatVector(q_cm_leader_)
                << " follower_offsets=(" << follower_joint1_offset_deg_ << ", "
                << follower_joint3_offset_deg_ << ")deg follower_q=" << formatVector(q_cm_follower_));
        }

        ROS_INFO_STREAM(
            "[coop_cm_runtime] lift_mode=" << lift_mode_
            << " transport_mode=" << transport_mode_
            << " z_tol=" << z_tolerance_
            << " rp_tol=" << roll_pitch_tolerance_
            << " ee_z_tol=" << ee_z_diff_tolerance_
            << " arm_q_tol=" << arm_posture_tolerance_);
    }

private:
    void loadParams() {
        pnh_.param<std::string>("ctrl_mode_topic", ctrl_mode_topic_, "/coop/ctrl_mode");
        pnh_.param<std::string>("leader_arm_posture_target_topic", leader_arm_topic_, "/leader/arm_posture_target");
        pnh_.param<std::string>("follower_arm_posture_target_topic", follower_arm_topic_, "/follower/arm_posture_target");
        pnh_.param<std::string>("leader_base_target_topic", leader_base_topic_, "/leader/base_target");
        pnh_.param<std::string>("follower_base_target_topic", follower_base_topic_, "/follower/base_target");
        pnh_.param<std::string>("cm_lift_delta_topic", lift_delta_topic_, "/coop/cm_lift_delta");
        pnh_.param<std::string>("cm_lift_ready_topic", seed_ready_topic_, "/coop/cm_lift_ready");
        pnh_.param<std::string>("cm_transport_ready_topic", transport_ready_topic_, "/coop/cm_transport_ready");
        pnh_.param<std::string>("ready_topic", lift_ready_topic_, "/coop/cm_lift_ready");
        pnh_.param<std::string>("metrics_topic", metrics_topic_, "/coop/grasp_tf_error");
        pnh_.param<std::string>("status_topic", lift_status_topic_, "/coop/cm_lift_status");
        pnh_.param<std::string>("joint_states_topic", joint_states_topic_,
                                "/coop_scene/mujoco_ros/mujoco_ros_interface/joint_states");
        pnh_.param<std::string>(
            "cm_seed_file",
            cm_seed_file_,
            "/home/ryoo/mmcm_ws/src/kimm_phri_panda_husky/config/holding_cm_seed.yaml");
        pnh_.param<std::string>("candidate_group", candidate_group_, "top_candidates");

        pnh_.param("enable_keyboard", enable_keyboard_, true);
        pnh_.param("keyboard_rate", keyboard_rate_hz_, 20.0);
        pnh_.param("lift_mode", lift_mode_, 238);
        pnh_.param("transport_mode", transport_mode_, 235);
        // CoopObjectGraspTargetNode also has a "publish_base_targets" option in
        // this integrated process. Use a CM-specific key so disabling pre-grasp
        // base motion does not silently disable the CM seed base target.
        pnh_.param("publish_cm_base_targets", publish_base_targets_, true);
        pnh_.param("publish_base_on_lift", publish_base_on_lift_, true);
        pnh_.param("publish_arm_posture_on_lift", publish_arm_posture_on_lift_, false);
        pnh_.param("publish_arm_posture_after_ready", publish_arm_posture_after_ready_, true);
        pnh_.param("publish_base_after_ready", publish_base_after_ready_, false);
        pnh_.param("publish_count", publish_count_default_, 8);
        pnh_.param("publish_period", publish_period_, 0.25);
        pnh_.param("candidate_index", candidate_index_, -1);
        pnh_.param("joint1_offset_deg", joint1_offset_deg_, 0.0);
        pnh_.param("joint3_offset_deg", joint3_offset_deg_, 0.0);
        pnh_.param("leader_joint1_offset_deg", leader_joint1_offset_deg_, joint1_offset_deg_);
        pnh_.param("leader_joint3_offset_deg", leader_joint3_offset_deg_, joint3_offset_deg_);
        pnh_.param("follower_joint1_offset_deg", follower_joint1_offset_deg_, joint1_offset_deg_);
        pnh_.param("follower_joint3_offset_deg", follower_joint3_offset_deg_, joint3_offset_deg_);

        pnh_.param("z_tolerance", z_tolerance_, 0.03);
        pnh_.param("roll_pitch_tolerance", roll_pitch_tolerance_, 0.10);
        pnh_.param("ee_z_diff_tolerance", ee_z_diff_tolerance_, 0.08);
        pnh_.param("xy_drift_tolerance", xy_drift_tolerance_, 0.12);
        pnh_.param("arm_posture_tolerance", arm_posture_tolerance_, 0.08);
        pnh_.param("metrics_timeout", metrics_timeout_, 0.50);
        pnh_.param("leader_arm_qpos_offset", leader_arm_qpos_offset_, 11);
        pnh_.param("follower_arm_qpos_offset", follower_arm_qpos_offset_, 31);
        pnh_.param("required_stable_samples", required_stable_samples_, 10);
        pnh_.param("lift_ready_rate", lift_ready_rate_hz_, 20.0);

        if (publish_period_ <= 0.0)
            publish_period_ = 0.25;
        if (!std::isfinite(metrics_timeout_) || metrics_timeout_ <= 0.0)
            metrics_timeout_ = 0.50;
    }

    void loadSeed() {
        const YAML::Node payload = YAML::LoadFile(cm_seed_file_);
        const YAML::Node initial_state = payload["initial_state"];
        if (!initial_state)
            throw std::runtime_error("initial_state is missing from " + cm_seed_file_);

        initial_bases_.leader = requirePose3(initial_state["leader_base_xyyaw"], "initial_state/leader_base_xyyaw");
        initial_bases_.follower = requirePose3(initial_state["follower_base_xyyaw"], "initial_state/follower_base_xyyaw");

        YAML::Node selected;
        if (candidate_index_ >= 0) {
            const YAML::Node candidates = payload[candidate_group_];
            if (!candidates || !candidates.IsSequence() ||
                static_cast<std::size_t>(candidate_index_) >= candidates.size()) {
                throw std::runtime_error("candidate_index is out of range for " + candidate_group_);
            }
            selected = candidates[static_cast<std::size_t>(candidate_index_)];
        } else {
            selected = payload["selected"];
        }
        if (!selected)
            throw std::runtime_error("selected CM seed is missing from " + cm_seed_file_);

        q_cm_raw_ = requireVector7(selected["q_sym"], "selected/q_sym");
        lift_delta_ = selected["lift_delta"].as<double>();

        if (selected["leader_base_xyyaw"] && selected["follower_base_xyyaw"]) {
            cm_bases_.leader = requirePose3(selected["leader_base_xyyaw"], "selected/leader_base_xyyaw");
            cm_bases_.follower = requirePose3(selected["follower_base_xyyaw"], "selected/follower_base_xyyaw");
        } else {
            cm_bases_.leader = requirePose3(selected["base_xyyaw"], "selected/base_xyyaw");
            cm_bases_.follower = {{cm_bases_.leader[0], initial_bases_.follower[1], cm_bases_.leader[2]}};
        }
    }

    std::vector<double> applyJoint13Offset(
        const std::vector<double>& q,
        double joint1_offset_deg,
        double joint3_offset_deg) const {
        std::vector<double> out = q;
        if (out.size() != 7)
            throw std::runtime_error("arm posture must have 7 joints");

        out[0] += degToRad(joint1_offset_deg);
        out[2] += degToRad(joint3_offset_deg);
        for (std::size_t i = 0; i < out.size(); ++i)
            out[i] = std::min(std::max(out[i], kArmLower[i]), kArmUpper[i]);
        return out;
    }

    void publishLiftDelta() {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(lift_delta_);
        lift_delta_pub_.publish(msg);
    }

    void publishTransportReady(bool ready) {
        std_msgs::Bool msg;
        msg.data = ready;
        transport_ready_pub_.publish(msg);
        transport_ready_ = ready;
    }

    void publishLiftReady(bool ready) {
        std_msgs::Bool msg;
        msg.data = ready;
        lift_ready_pub_.publish(msg);
    }

    void queueTargets(
        bool has_q,
        const std::vector<double>& leader_q,
        const std::vector<double>& follower_q,
        bool has_bases,
        const BasePair& bases,
        const std::string& label) {
        has_pending_q_leader_ = has_q;
        has_pending_q_follower_ = has_q;
        pending_q_leader_ = has_q ? leader_q : std::vector<double>();
        pending_q_follower_ = has_q ? follower_q : std::vector<double>();
        has_pending_bases_ = has_bases;
        pending_bases_ = bases;
        pending_label_ = label;
        pending_count_ = publish_count_default_;

        ROS_INFO_STREAM(
            "[coop_cm_runtime] queue " << label
            << ": leader_arm=" << (has_pending_q_leader_ ? formatVector(pending_q_leader_) : "SE(3)-only")
            << " follower_arm=" << (has_pending_q_follower_ ? formatVector(pending_q_follower_) : "SE(3)-only")
            << " base=" << (has_pending_bases_ ? formatBases(pending_bases_) : "unchanged")
            << " repeat=" << pending_count_
            << " period=" << std::fixed << std::setprecision(2) << publish_period_ << "s");
        publishTargetsOnce();
    }

    void publishTargetsOnce() {
        if (has_pending_q_leader_) {
            std_msgs::Float64MultiArray msg;
            msg.data = pending_q_leader_;
            leader_arm_pub_.publish(msg);
        }
        if (has_pending_q_follower_) {
            std_msgs::Float64MultiArray msg;
            msg.data = pending_q_follower_;
            follower_arm_pub_.publish(msg);
        }
        if (publish_base_targets_ && has_pending_bases_) {
            leader_base_pub_.publish(toTransformMsg(pending_bases_.leader));
            follower_base_pub_.publish(toTransformMsg(pending_bases_.follower));
        }
    }

    void targetTimerCallback(const ros::TimerEvent&) {
        publishLiftDelta();
        if (pending_count_ <= 0)
            return;

        publishTargetsOnce();
        --pending_count_;
        if (pending_count_ <= 0 && transport_ready_after_pending_) {
            transport_ready_after_pending_ = false;
            publishTransportReady(true);
            ROS_INFO_STREAM(
                "[coop_cm_runtime] CM posture settling complete; transport ready published on "
                << transport_ready_topic_);
        }
    }

    void modeCallback(const std_msgs::Int32::ConstPtr& msg) {
        const int mode = msg->data;

        if (mode == lift_mode_) {
            lift_request_active_ = true;
            post_ready_posture_sent_ = false;
            transport_ready_after_pending_ = false;
            publishTransportReady(false);
            publishLiftDelta();

            const bool has_bases = publish_base_on_lift_;
            const bool has_q = publish_arm_posture_on_lift_;
            if (!has_q && !has_bases) {
                if (publish_arm_posture_after_ready_) {
                    ROS_INFO_STREAM(
                        "[coop_cm_runtime] mode " << lift_mode_
                        << ": lift_delta published; waiting for lift-ready before publishing CM posture reference");
                } else {
                    ROS_INFO_STREAM(
                        "[coop_cm_runtime] mode " << lift_mode_
                        << ": lift_delta published; arm/base targets left to object-grasp target controller");
                }
            } else {
                queueTargets(has_q, q_cm_leader_, q_cm_follower_, has_bases, cm_bases_, "holding CM lift");
            }

            lift_active_ = true;
            lift_ready_ = false;
            stable_count_ = 0;
            baseline_valid_ = false;
            target_z_valid_ = false;
            publishLiftReady(false);
            captureBaseline();
            ROS_INFO_STREAM(
                "[coop_cm_runtime] CM lift requested; waiting for settled lift before mode "
                << transport_mode_);
        } else if (mode == transport_mode_ && lift_active_) {
            if (!lift_ready_) {
                ROS_WARN_STREAM("[coop_cm_runtime] mode " << transport_mode_ << " requested before CM lift ready");
            } else {
                // Readiness is an entry gate. Once transport starts, q_sym and
                // object xyz intentionally leave the lift-settled tolerance.
                lift_active_ = false;
                lift_request_active_ = false;
                ROS_INFO_STREAM("[coop_cm_runtime] mode " << transport_mode_
                                << " accepted; lift readiness gate consumed");
            }
        }
    }

    void keyboardTimerCallback(const ros::TimerEvent&) {
        if (!terminal_.valid())
            return;

        const std::map<char, int> key_to_mode = {
            {'h', 1},
            {'p', 24},
            {'c', 21},
            {'t', 211},
            {'y', 238},
            {'0', 235},
        };

        char key = 0;
        while (readKey(&key)) {
            if (key == '?') {
                printKeyboardHelp();
                continue;
            }

            const auto it = key_to_mode.find(key);
            if (it == key_to_mode.end()) {
                if (std::isprint(static_cast<unsigned char>(key)))
                    ROS_INFO("[coop_cm_runtime] unknown key '%c' (? for help)", key);
                continue;
            }

            std_msgs::Int32 msg;
            msg.data = it->second;
            mode_pub_.publish(msg);
            ROS_INFO("[coop_cm_runtime] key '%c' -> ctrl_mode %d", key, msg.data);
        }
    }

    void readyCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (!msg->data || !lift_request_active_ || post_ready_posture_sent_) {
            return;
        }

        post_ready_posture_sent_ = true;
        if (!publish_arm_posture_after_ready_) {
            publishTransportReady(true);
            ROS_INFO_STREAM(
                "[coop_cm_runtime] CM lift settled; transport ready published on "
                << transport_ready_topic_);
            return;
        }

        transport_ready_after_pending_ = true;
        queueTargets(
            true,
            q_cm_leader_,
            q_cm_follower_,
            publish_base_after_ready_,
            cm_bases_,
            "holding CM posture after lift ready");
    }

    void liftDeltaCallback(const std_msgs::Float32::ConstPtr& msg) {
        const double value = msg->data;
        if (std::isfinite(value))
            lift_delta_ = value;
    }

    void metricsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        if (msg->data.size() < 19)
            return;
        for (std::size_t i = 0; i < 19; ++i) {
            if (!std::isfinite(msg->data[i])) {
                ROS_WARN_THROTTLE(1.0, "[coop_cm_runtime] rejected non-finite grasp metric");
                return;
            }
        }

        latest_metrics_.assign(msg->data.begin(), msg->data.end());
        has_metrics_ = true;
        last_metrics_wall_time_ = ros::WallTime::now();
        if (lift_active_ && !baseline_valid_)
            captureBaseline();
    }

    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        const int required = std::max(leader_arm_qpos_offset_, follower_arm_qpos_offset_) + 7;
        if (static_cast<int>(msg->position.size()) < required)
            return;
        latest_leader_q_.assign(msg->position.begin() + leader_arm_qpos_offset_,
                                msg->position.begin() + leader_arm_qpos_offset_ + 7);
        latest_follower_q_.assign(msg->position.begin() + follower_arm_qpos_offset_,
                                  msg->position.begin() + follower_arm_qpos_offset_ + 7);
        has_arm_state_ = true;
    }

    double maxArmPostureError() const {
        if (!has_arm_state_ || latest_leader_q_.size() != 7 || latest_follower_q_.size() != 7)
            return std::numeric_limits<double>::infinity();
        double max_error = 0.0;
        for (std::size_t i = 0; i < 7; ++i) {
            max_error = std::max(max_error, std::abs(latest_leader_q_[i] - q_cm_leader_[i]));
            max_error = std::max(max_error, std::abs(latest_follower_q_[i] - q_cm_follower_[i]));
        }
        return max_error;
    }

    void captureBaseline() {
        if (!has_metrics_)
            return;

        baseline_z_ = latest_metrics_[4];
        baseline_x_ = latest_metrics_[16];
        baseline_y_ = latest_metrics_[17];
        target_z_ = baseline_z_ + lift_delta_;
        baseline_valid_ = true;
        target_z_valid_ = true;
        ROS_INFO_STREAM(
            "[coop_cm_runtime] captured lift baseline xyz=["
            << std::fixed << std::setprecision(3)
            << baseline_x_ << " " << baseline_y_ << " " << baseline_z_
            << "] target_z=" << target_z_ << " delta=" << lift_delta_);
    }

    void liftTimerCallback(const ros::TimerEvent&) {
        if (!lift_active_ || !has_metrics_ || !target_z_valid_)
            return;

        const bool metrics_fresh = !last_metrics_wall_time_.isZero() &&
            (ros::WallTime::now() - last_metrics_wall_time_).toSec() <= metrics_timeout_;
        if (!metrics_fresh) {
            stable_count_ = 0;
            if (lift_ready_ || transport_ready_) {
                lift_ready_ = false;
                publishLiftReady(false);
                publishTransportReady(false);
                post_ready_posture_sent_ = false;
                transport_ready_after_pending_ = false;
                ROS_WARN_THROTTLE(
                    1.0,
                    "[coop_cm_runtime] lift readiness revoked: grasp metrics are stale");
            }
            return;
        }

        const double object_z = latest_metrics_[4];
        const double roll = std::abs(latest_metrics_[10]);
        const double pitch = std::abs(latest_metrics_[11]);
        const double ee_z_diff = std::abs(latest_metrics_[15]);
        const double total_xy_drift = latest_metrics_[18];
        const double xy_drift = baseline_valid_
                                    ? std::hypot(latest_metrics_[16] - baseline_x_, latest_metrics_[17] - baseline_y_)
                                    : total_xy_drift;
        const double z_error = std::abs(object_z - target_z_);
        const double arm_posture_error = maxArmPostureError();
        const bool arm_posture_required = publish_arm_posture_on_lift_;
        const bool arm_posture_ok = !arm_posture_required ||
                                    (std::isfinite(arm_posture_error) &&
                                     arm_posture_error <= arm_posture_tolerance_);

        const bool stable =
            z_error <= z_tolerance_ &&
            roll <= roll_pitch_tolerance_ &&
            pitch <= roll_pitch_tolerance_ &&
            ee_z_diff <= ee_z_diff_tolerance_ &&
            xy_drift <= xy_drift_tolerance_ &&
            arm_posture_ok;

        stable_count_ = stable ? stable_count_ + 1 : 0;
        const bool new_ready = stable_count_ >= required_stable_samples_;
        if (new_ready != lift_ready_) {
            lift_ready_ = new_ready;
            publishLiftReady(lift_ready_);
            if (lift_ready_) {
                ROS_INFO_STREAM(
                    "[coop_cm_runtime] READY for mode " << transport_mode_
                    << ": z=" << std::fixed << std::setprecision(3) << object_z
                    << " target=" << target_z_
                    << " roll=" << roll
                    << " pitch=" << pitch
                    << " ee_z_diff=" << ee_z_diff
                    << " arm_q_max_error=" << arm_posture_error);
            } else {
                publishTransportReady(false);
                post_ready_posture_sent_ = false;
                transport_ready_after_pending_ = false;
                ROS_WARN_STREAM(
                    "[coop_cm_runtime] lift readiness revoked: state left tolerance");
            }
        }

        std_msgs::Float32MultiArray status;
        status.data = {
            lift_ready_ ? 1.0f : 0.0f,
            static_cast<float>(object_z),
            static_cast<float>(target_z_),
            static_cast<float>(z_error),
            static_cast<float>(roll),
            static_cast<float>(pitch),
            static_cast<float>(ee_z_diff),
            static_cast<float>(xy_drift),
            static_cast<float>(stable_count_),
            static_cast<float>(total_xy_drift),
            static_cast<float>(arm_posture_error),
            arm_posture_ok ? 1.0f : 0.0f};
        lift_status_pub_.publish(status);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::string ctrl_mode_topic_;
    std::string leader_arm_topic_;
    std::string follower_arm_topic_;
    std::string leader_base_topic_;
    std::string follower_base_topic_;
    std::string lift_delta_topic_;
    std::string seed_ready_topic_;
    std::string transport_ready_topic_;
    std::string lift_ready_topic_;
    std::string metrics_topic_;
    std::string lift_status_topic_;
    std::string joint_states_topic_;
    std::string cm_seed_file_;
    std::string candidate_group_;

    int lift_mode_ = 238;
    int transport_mode_ = 235;
    int publish_count_default_ = 8;
    int pending_count_ = 0;
    int candidate_index_ = -1;
    int required_stable_samples_ = 10;
    int stable_count_ = 0;
    int leader_arm_qpos_offset_ = 11;
    int follower_arm_qpos_offset_ = 31;

    double publish_period_ = 0.25;
    double joint1_offset_deg_ = 0.0;
    double joint3_offset_deg_ = 0.0;
    double leader_joint1_offset_deg_ = 0.0;
    double leader_joint3_offset_deg_ = 0.0;
    double follower_joint1_offset_deg_ = 0.0;
    double follower_joint3_offset_deg_ = 0.0;
    double lift_delta_ = 0.12;
    double lift_ready_rate_hz_ = 20.0;
    double z_tolerance_ = 0.03;
    double roll_pitch_tolerance_ = 0.10;
    double ee_z_diff_tolerance_ = 0.08;
    double xy_drift_tolerance_ = 0.12;
    double arm_posture_tolerance_ = 0.08;
    double metrics_timeout_ = 0.50;
    double baseline_z_ = 0.0;
    double baseline_x_ = 0.0;
    double baseline_y_ = 0.0;
    double target_z_ = 0.0;
    double keyboard_rate_hz_ = 20.0;

    bool enable_keyboard_ = true;
    bool publish_base_targets_ = true;
    bool publish_base_on_lift_ = true;
    bool publish_arm_posture_on_lift_ = false;
    bool publish_arm_posture_after_ready_ = true;
    bool publish_base_after_ready_ = false;
    bool lift_request_active_ = false;
    bool post_ready_posture_sent_ = false;
    bool transport_ready_after_pending_ = false;
    bool has_pending_q_leader_ = false;
    bool has_pending_q_follower_ = false;
    bool has_pending_bases_ = false;
    bool lift_active_ = false;
    bool lift_ready_ = false;
    bool transport_ready_ = false;
    bool has_metrics_ = false;
    bool baseline_valid_ = false;
    bool target_z_valid_ = false;
    bool has_arm_state_ = false;

    std::vector<double> q_cm_raw_;
    std::vector<double> q_cm_leader_;
    std::vector<double> q_cm_follower_;
    std::vector<double> pending_q_leader_;
    std::vector<double> pending_q_follower_;
    std::vector<float> latest_metrics_;
    std::vector<double> latest_leader_q_;
    std::vector<double> latest_follower_q_;
    std::string pending_label_;
    BasePair initial_bases_;
    BasePair cm_bases_;
    BasePair pending_bases_;
    ros::WallTime last_metrics_wall_time_;

    ros::Publisher leader_arm_pub_;
    ros::Publisher follower_arm_pub_;
    ros::Publisher leader_base_pub_;
    ros::Publisher follower_base_pub_;
    ros::Publisher lift_delta_pub_;
    ros::Publisher transport_ready_pub_;
    ros::Publisher lift_ready_pub_;
    ros::Publisher lift_status_pub_;
    ros::Publisher mode_pub_;
    ros::Subscriber mode_sub_;
    ros::Subscriber ready_sub_;
    ros::Subscriber metrics_sub_;
    ros::Subscriber joint_states_sub_;
    ros::Subscriber lift_delta_sub_;
    ros::Timer target_timer_;
    ros::Timer lift_timer_;
    ros::Timer keyboard_timer_;
    TerminalRawMode terminal_;
};
ros::Duration periodFromHz(double hz) {
    if (!std::isfinite(hz) || hz <= 0.0)
        hz = 20.0;
    return ros::Duration(1.0 / hz);
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "coop_phri_simul");

    try {
        CoopGraspTransitionNode grasp_transition;
        CoopGraspTfMonitor grasp_monitor;
        CoopObjectGraspTargetNode object_targets;
        CoopJointSetMerger joint_merger;
        CoopCmRuntimeNode cm_runtime;

        ros::NodeHandle nh;
        ros::Timer monitor_timer = nh.createTimer(
            periodFromHz(grasp_monitor.rateHz()),
            [&](const ros::TimerEvent&) { grasp_monitor.update(); });
        ros::Timer target_timer = nh.createTimer(
            periodFromHz(object_targets.rateHz()),
            [&](const ros::TimerEvent&) { object_targets.update(); });
        ros::Timer merger_timer = nh.createTimer(
            periodFromHz(joint_merger.rateHz()),
            [&](const ros::TimerEvent&) { joint_merger.update(); });

        ROS_INFO("[coop_phri_simul] integrated cooperative transport runtime started");
        ros::spin();
    } catch (const std::exception& ex) {
        ROS_FATAL_STREAM("[coop_phri_simul] " << ex.what());
        return 1;
    }

    return 0;
}
