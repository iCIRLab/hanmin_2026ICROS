#include "kimm_phri_panda_husky/panda_husky_hqp.h"

#include <algorithm>
#include <cmath>
#include <sstream>

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace kimmhqp;
using namespace kimmhqp::trajectory;
using namespace kimmhqp::math;
using namespace kimmhqp::tasks;
using namespace kimmhqp::solver;
using namespace kimmhqp::robot;
using namespace kimmhqp::contacts;

namespace RobotController{
    FrankaHuskyWrapper::FrankaHuskyWrapper(const std::string & robot_node, const bool & issimulation, const bool & ismobile, const bool & isrobotiq, ros::NodeHandle & node)
    : robot_node_(robot_node), issimulation_(issimulation), ismobile_(ismobile), isrobotiq_(isrobotiq), n_node_(node)
    {
        time_ = 0.;
        cnt_ = 0;

        mode_change_ = false;
        ctrl_mode_ = 0;
    }

    void FrankaHuskyWrapper::initialize(){
        // Robot for pinocchio
        string model_path, urdf_name;
        n_node_.getParam("/" + robot_node_ +"/robot_urdf_path", model_path);
        n_node_.getParam("/" + robot_node_ +"/robot_urdf", urdf_name);        //"panda_arm_hand_l.urdf" w/o mobile, "husky_panda_hand.urdf" w/ mobile

        vector<string> package_dirs;
        package_dirs.push_back(model_path);
        string urdfFileName = package_dirs[0] + urdf_name;

        robot_ = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, ismobile_, false); //first false : w/o mobile, true : w/ mobile
        model_ = robot_->model();

        //nq_/nv_/na_ is # of joint w.r.t pinocchio model ("panda_arm_hand_l.urdf"), so there is no gripper joints
        nq_ = robot_->nq(); //12 : odom (3 - x,y,theta) + husky (2) + franka (7)
        nv_ = robot_->nv(); //12
        na_ = robot_->na(); //9  : nv-3, odom dof eliminated

        // State (for pinocchio)
        state_.q_.setZero(nq_);
        state_.v_.setZero(nv_);
        state_.dv_.setZero(nv_);
        state_.torque_.setZero(na_);
        state_.tau_.setZero(na_);

        // tsid
        tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot_);
        tsid_->computeProblemData(time_, state_.q_, state_.v_);
        data_ = tsid_->data();

        // tasks
        postureTask_ = std::make_shared<TaskJointPosture>("task-posture", *robot_);

        VectorXd posture_gain;
        if (ismobile_) posture_gain.resize(na_-2);
        else           posture_gain.resize(na_);

        if (!issimulation_) //for real
            // posture_gain << 200., 200., 200., 200., 200., 200., 200.;
            posture_gain << 100., 100., 100., 200., 200., 200., 200.;
        else // for simulation
            posture_gain << 10000., 10000., 10000., 10000., 10000., 10000., 10000.;

        posture_gain_default_ = posture_gain;
        postureTask_->Kp(posture_gain);
        postureTask_->Kd(2.0*postureTask_->Kp().cwiseSqrt());

        //////////////////// EE offset ////////////////////////////////////////
        // link7 대신 실제 grasp/control frame을 EE task 기준으로 사용한다.
        if (isrobotiq_) joint7_to_finger_ = 0.247; //0.247 (z-axis) = 0.222(distance from link7 to left_inner_finger) + 0.025(finger center length)
        else            joint7_to_finger_ = 0.2054; //0.2054 (z-axis) = 0.107(distance from link7 to EE) + 0.0584(hand length) + 0.04(finger length)
        this->eeoffset_update();

        // Retained for the public trajectory timing helper.
        traj_length_in_time_ = 10.0;
        ///////////////////////////////////////////////////////////////////////

        VectorXd ee_gain(6);
        if (!issimulation_) { //for real
            ee_gain << 100., 100., 100., 400., 400., 600.;
        }
        else { //for simulation
            // ee_gain << 500., 500., 500., 800., 800., 1000.;
            ee_gain << 1000., 1000., 1000., 2000., 2000., 2000.;
        }

        // 물체 자세 안정성을 위해 EE orientation gain을 필요 시 높인다.
        double ee_pos_gain = ee_gain(0);
        double ee_ori_gain = ee_gain(3);
        double ee_kd_ratio = 1.0;

        // ICROS2026 협업 운반 기본값. compact launch는 숫자 파라미터를 생략하고 이 값을 쓴다.
        const bool is_coop_transport_robot =
            (robot_node_ == "ns1" || robot_node_ == "ns0");
        if (issimulation_ && is_coop_transport_robot) {
            ee_ori_gain = 3000.0;

            mode235_path_p1_y_ = 0.30;
            mode235_path_p1_theta_ = 20.0;
            mode235_path_p1_duration_ = 15.0;
            mode235_path_p2_y_ = 0.30;
            mode235_path_p2_theta_ = 0.0;
            mode235_path_p2_duration_ = 15.0;
            mode235_path_p3_y_ = 0.00;
            mode235_path_p3_theta_ = -20.0;
            mode235_path_p3_duration_ = 15.0;
            mode235_path_p4_y_ = 0.00;
            mode235_path_p4_theta_ = 0.0;
            mode235_path_p4_duration_ = 15.0;
            mode235_path_p5_y_ = 0.00;
            mode235_path_p5_theta_ = 0.0;
            mode235_path_p5_duration_ = 15.0;
            mode235_pre_straight_x_ = 3.00;
            mode235_pre_straight_duration_ = 15.0;
            mode235_path_relative_x_ = true;
            mode235_path_relative_y_ = true;
            mode235_phase_xy_tolerance_ = 0.20;
            mode235_phase3_xy_tolerance_ = 0.20;
            mode235_phase_yaw_tolerance_deg_ = 20.0;
            mode235_follower_posture_weight_ = 0.10;

            if (robot_node_ == "ns1") {
                mobile_xy_weight_ = 4.0;
                mobile_yaw_weight_ = 8.0;
                mobile_xy_kp_ = 100.0;
                mobile_yaw_kp_ = 400.0;
                mode235_leader_posture_weight_ = 0.30;
                mode235_leader_ee_pos_kp_ = 120.0;
                mode235_leader_ee_ori_kp_ = 180.0;
                mode235_path_straight_path_ = true;
                mode235_path_straight_x_scale_ = 1.0;
                mode235_path_straight_p1_duration_ = 13.0;
                mode235_path_straight_p2_duration_ = 10.0;
                mode235_path_straight_p3_duration_ = 10.0;
                mode235_path_straight_p4_duration_ = 10.0;
                mode235_path_p1_x_ = 1.10;
                mode235_path_p2_x_ = 1.85;
                mode235_path_p3_x_ = 2.25;
                mode235_path_p4_x_ = 3.00;
                mode235_path_p5_x_ = 6.35;
            } else {
                mobile_xy_weight_ = 4.0;
                mobile_yaw_weight_ = 8.0;
                mobile_xy_kp_ = 180.0;
                mobile_yaw_kp_ = 450.0;
                mode235_path_straight_path_ = false;
                mode235_path_p1_x_ = 1.35;
                mode235_path_p2_x_ = 2.00;
                mode235_path_p3_x_ = 2.65;
                mode235_path_p4_x_ = 3.30;
                mode235_path_p5_x_ = 6.35;
            }
        }

        n_node_.getParam("/" + robot_node_ + "/ee_pos_gain", ee_pos_gain);
        n_node_.getParam("/" + robot_node_ + "/ee_ori_gain", ee_ori_gain);
        n_node_.getParam("/" + robot_node_ + "/ee_kd_ratio", ee_kd_ratio);
        ee_gain << ee_pos_gain, ee_pos_gain, ee_pos_gain, ee_ori_gain, ee_ori_gain, ee_ori_gain;

        n_node_.getParam("/" + robot_node_ + "/mobile_yaw_weight", mobile_yaw_weight_);
        n_node_.getParam("/" + robot_node_ + "/mobile_xy_weight", mobile_xy_weight_);
        n_node_.getParam("/" + robot_node_ + "/mobile_yaw_kp", mobile_yaw_kp_);
        n_node_.getParam("/" + robot_node_ + "/mobile_xy_kp", mobile_xy_kp_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_p1_x", mode235_path_p1_x_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_p1_y", mode235_path_p1_y_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_p1_theta", mode235_path_p1_theta_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_p1_duration", mode235_path_p1_duration_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_p2_x", mode235_path_p2_x_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_p2_y", mode235_path_p2_y_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_p2_theta", mode235_path_p2_theta_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_p2_duration", mode235_path_p2_duration_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_p3_x", mode235_path_p3_x_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_p3_y", mode235_path_p3_y_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_p3_theta", mode235_path_p3_theta_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_p3_duration", mode235_path_p3_duration_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_p4_x", mode235_path_p4_x_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_p4_y", mode235_path_p4_y_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_p4_theta", mode235_path_p4_theta_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_p4_duration", mode235_path_p4_duration_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_p5_x", mode235_path_p5_x_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_p5_y", mode235_path_p5_y_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_p5_theta", mode235_path_p5_theta_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_p5_duration", mode235_path_p5_duration_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_straight_path", mode235_path_straight_path_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_straight_y", mode235_path_straight_y_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_straight_theta", mode235_path_straight_theta_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_straight_x_scale", mode235_path_straight_x_scale_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_straight_p1_duration", mode235_path_straight_p1_duration_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_straight_p2_duration", mode235_path_straight_p2_duration_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_straight_p3_duration", mode235_path_straight_p3_duration_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_straight_p4_duration", mode235_path_straight_p4_duration_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_relative_x", mode235_path_relative_x_);
        n_node_.getParam("/" + robot_node_ + "/mode235_path_relative_y", mode235_path_relative_y_);
        n_node_.getParam("/" + robot_node_ + "/mode235_pre_straight_x", mode235_pre_straight_x_);
        n_node_.getParam("/" + robot_node_ + "/mode235_pre_straight_duration", mode235_pre_straight_duration_);
        n_node_.getParam("/" + robot_node_ + "/mode235_leader_posture_weight", mode235_leader_posture_weight_);
        n_node_.getParam("/" + robot_node_ + "/mode235_leader_ee_pos_kp", mode235_leader_ee_pos_kp_);
        n_node_.getParam("/" + robot_node_ + "/mode235_leader_ee_ori_kp", mode235_leader_ee_ori_kp_);
        n_node_.getParam("/" + robot_node_ + "/mode235_follower_posture_weight", mode235_follower_posture_weight_);
        n_node_.getParam("/" + robot_node_ + "/mode235_follower_ee_pos_kp", mode235_follower_ee_pos_kp_);
        n_node_.getParam("/" + robot_node_ + "/mode235_follower_ee_ori_kp", mode235_follower_ee_ori_kp_);
        n_node_.getParam("/" + robot_node_ + "/mode235_follower_ee_kd_ratio", mode235_follower_ee_kd_ratio_);
        n_node_.getParam("/" + robot_node_ + "/mode235_phase_xy_tolerance", mode235_phase_xy_tolerance_);
        n_node_.getParam("/" + robot_node_ + "/mode235_phase3_xy_tolerance", mode235_phase3_xy_tolerance_);
        n_node_.getParam("/" + robot_node_ + "/mode235_phase_yaw_tolerance_deg", mode235_phase_yaw_tolerance_deg_);
        mode235_phase_ready_pub_ =
            n_node_.advertise<std_msgs::Int16>("/" + robot_node_ + "/mode235_phase_ready", 10);
        const std::string peer_node =
            (robot_node_ == "ns0") ? "ns1" : ((robot_node_ == "ns1") ? "ns0" : "");
        if (!peer_node.empty()) {
            mode235_peer_ready_sub_ =
                n_node_.subscribe("/" + peer_node + "/mode235_phase_ready", 10,
                                  &FrankaHuskyWrapper::mode235PeerReadyCallback, this);
        }
        if (is_coop_transport_robot) {
            mode235_grasp_error_sub_ =
                n_node_.subscribe("/coop/grasp_tf_error", 10,
                                  &FrankaHuskyWrapper::mode235GraspErrorCallback, this);
        }

        eeTask_ = std::make_shared<TaskSE3Equality>("task-se3", *robot_, "panda_joint7", ee_offset_); //here, ee_offset_ is applied to current pos value
        eeTask_->Kp(ee_gain*Vector::Ones(6));
        // eeTask_->Kd(2.0*eeTask_->Kp().cwiseSqrt());
        eeTask_->Kd(ee_kd_ratio*eeTask_->Kp().cwiseSqrt());

        // elbow z task: EE task를 유지한 채 null-space에서 팔 자세를 조정한다.
        elbowTask_ = std::make_shared<TaskSE3Equality>("task-elbow", *robot_, "panda_joint4", Eigen::Vector3d::Zero());
        elbowTask_->Kp(400.0*Vector::Ones(6));
        elbowTask_->Kd(1.0*elbowTask_->Kp().cwiseSqrt());
        { Vector emask(6); emask << 0,0,1,0,0,0; elbowTask_->setMask(emask); }

        torqueBoundsTask_ = std::make_shared<TaskJointBounds>("task-torque-bounds", *robot_);
        Vector dq_max = 500000.0*Vector::Ones(na_);
        dq_max(0) = 500.; //?
        dq_max(1) = 500.; //?
        Vector dq_min = -dq_max;
        torqueBoundsTask_->setJointBounds(dq_min, dq_max);

        if (ismobile_) {
            mobileTask_ = std::make_shared<TaskMobileEquality>("task-mobile", *robot_, true); //ori
            mobileTask_->Kp(50.0*Vector3d::Ones());
            mobileTask_->Kd(2.5*mobileTask_->Kp().cwiseSqrt());

            mobileTask2_ = std::make_shared<TaskMobileEquality>("task-mobile2", *robot_, false);
            mobileTask2_->Kp(100.0*Vector3d::Ones());
            mobileTask2_->Kd(2.5*mobileTask2_->Kp().cwiseSqrt());
        }

        // trajecotries
        sampleEE_.resize(12, 6); //12=3(translation)+9(rotation matrix), 6=3(translation)+3(rotation)

        if (ismobile_) samplePosture_.resize(na_-2); //na_=9 (husky 2 + franka 7)
        else           samplePosture_.resize(na_); //na_=7 franka 7

        trajPosture_Cubic_ = std::make_shared<TrajectoryEuclidianCubic>("traj_posture");
        trajPosture_Constant_ = std::make_shared<TrajectoryEuclidianConstant>("traj_posture_constant");
        trajPosture_Timeopt_ = std::make_shared<TrajectoryEuclidianTimeopt>("traj_posture_timeopt");

        trajEE_Cubic_ = std::make_shared<TrajectorySE3Cubic>("traj_ee");
        trajEE_Constant_ = std::make_shared<TrajectorySE3Constant>("traj_ee_constant");
        Vector3d Maxvel_ee = Vector3d::Ones()*0.2;
        Vector3d Maxacc_ee = Vector3d::Ones()*0.2;
        trajEE_Timeopt_ = std::make_shared<TrajectorySE3Timeopt>("traj_ee_timeopt", Maxvel_ee, Maxacc_ee);

        if (ismobile_) {
            sampleMobile_.resize(12, 6);
            trajMobile_Cubic_ = std::make_shared<TrajectorySE3Cubic>("traj_mobile");
            trajMobile_Constant_ = std::make_shared<TrajectorySE3Constant>("traj_mobile_constant");
            Vector3d Maxvel_base = Vector3d::Ones()*1.0;
            Vector3d Maxacc_base = Vector3d::Ones()*1.0;
            trajMobile_Timeopt_ = std::make_shared<TrajectorySE3Timeopt>("traj_mobile_timeopt", Maxvel_base, Maxacc_base);
        }

        // Initialize the home posture once so any later mode can safely reuse q_ref_.
        q_ref_.setZero(7);
        q_ref_(0) = 0.0;
        q_ref_(1) = 0.0;
        q_ref_(3) = -M_PI / 2.0;
        q_ref_(5) = M_PI / 2.0;
        if (isrobotiq_) q_ref_(6) = -M_PI / 2.0;
        else            q_ref_(6) = -M_PI / 4.0;

        // solver
        solver_ = SolverHQPFactory::createNewSolver(SOLVER_HQP_QPOASES, "qpoases");

        reset_control_ = true;

        // Desired inertia for the active HQP tasks.
        eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
        if (ismobile_) {
            mobileTask_->setDesiredinertia(MatrixXd::Identity(6,6));
            mobileTask2_->setDesiredinertia(MatrixXd::Identity(6,6));
        }
    }

    void FrankaHuskyWrapper::eeoffset_update(){
        ee_offset_ = Vector3d(0.0, 0.0, joint7_to_finger_); //w.r.t joint7

        T_offset_.setIdentity();
        T_offset_.translation(ee_offset_);

        Adj_mat_.resize(6,6);
        Adj_mat_.setIdentity();
        Adj_mat_.topRightCorner(3,3) = -1 * skew_matrix(ee_offset_); //due to "A cross B = -B cross A"
    }

    void FrankaHuskyWrapper::franka_update(const sensor_msgs::JointState& msg){ //for simulation (mujoco)
        // mujoco callback msg
        // msg.position : 7(odom) + 4(wheel) + 7(joint) + 2(gripper)
        // msg.velocity : 6(odom) + 4(wheel) + 7(joint) + 2(gripper)

        assert(issimulation_);
        for (int i=0; i< 7; i++){
            if (ismobile_) {
                state_.q_(i+5) = msg.position[i+11];
                state_.v_(i+5) = msg.velocity[i+10];
            }
            else {
                state_.q_(i) = msg.position[i];
                state_.v_(i) = msg.velocity[i];
            }
        }
    }
    void FrankaHuskyWrapper::franka_update(const Vector7d& q, const Vector7d& qdot){ //for experiment
        assert(!issimulation_);
        state_.q_.tail(7) = q;
        state_.v_.tail(7) = qdot;
    }

    void FrankaHuskyWrapper::franka_update(const Vector7d& q, const Vector7d& qdot, const Vector7d& tau){ //for experiment, use pinocchio::aba
        assert(!issimulation_);
        state_.q_.tail(7) = q;
        state_.v_.tail(7) = qdot;

        state_.tau_.tail(7) = tau;
    }

    void FrankaHuskyWrapper::husky_update(const sensor_msgs::JointState& msg){ //for simulation (mujoco)
        // mujoco callback msg
        // msg.position : 7(odom) + 4(wheel) + 7(joint) + 2(gripper)
        // msg.velocity : 6(odom) + 4(wheel) + 7(joint) + 2(gripper)

        assert(issimulation_);
        for (int i=0; i<2; i++){ //odom x, y
            state_.q_(i) = msg.position[i];
            state_.v_(i) = msg.velocity[i];
        }

        double theta = atan2(2.* (msg.position[5] * msg.position[4] + msg.position[6] * msg.position[3]), 1- 2.*(pow( msg.position[6], 2) + pow(msg.position[5], 2))); //odom yaw
        state_.q_(2) = theta;
        state_.v_(2) = msg.velocity[5];

        // only for front wheel (not used)
        state_.q_(3) = msg.position[7];
        state_.q_(4) = msg.position[8];
        state_.v_(3) = msg.velocity[6];
        state_.v_(4) = msg.velocity[7];
    }
    void FrankaHuskyWrapper::husky_update(const Vector3d& base_pos, const Vector3d& base_vel, const Vector2d& wheel_pos, const Vector2d& wheel_vel){ //for experiment
        assert(!issimulation_);
        for (int i=0; i<3; i++){
            state_.q_(i) = base_pos(i);
            state_.v_(i) = base_vel(i);
        }
        for (int i=0; i<2; i++){
            state_.q_(i+3) = wheel_pos(i);
            state_.v_(i+3) = wheel_vel(i);
        }
    }

    void FrankaHuskyWrapper::Fext_update(const Vector6d& Fext){
        if (!Fext.allFinite()) {
            Fext_.setZero();
            has_fext_ = false;
            ROS_WARN_THROTTLE(1.0, "[%s] rejected non-finite external wrench", robot_node_.c_str());
            return;
        }
        Fext_ = Fext;
        has_fext_ = true;
    }

    void FrankaHuskyWrapper::mode235PeerReadyCallback(const std_msgs::Int16::ConstPtr & msg){
        mode235_peer_ready_phase_ = std::max(mode235_peer_ready_phase_, static_cast<int>(msg->data));
    }

    void FrankaHuskyWrapper::mode235GraspErrorCallback(const std_msgs::Float32MultiArray::ConstPtr & msg){
        if (msg->data.size() <= 14)
            return;
        const double leader_ee_z = static_cast<double>(msg->data[13]);
        const double follower_ee_z = static_cast<double>(msg->data[14]);
        if (!std::isfinite(leader_ee_z) || !std::isfinite(follower_ee_z))
            return;
        mode235_latest_leader_ee_z_ = leader_ee_z;
        mode235_latest_follower_ee_z_ = follower_ee_z;
        mode235_has_lift_height_metrics_ = true;
    }

    double FrankaHuskyWrapper::mode235CurrentPostureWeight() const {
        return mode235_path_straight_path_ ? mode235_leader_posture_weight_ : mode235_follower_posture_weight_;
    }

    void FrankaHuskyWrapper::configureDefaultPostureTask(){
        if (posture_gain_default_.size() == 0)
            return;
        Vector posture_mask = Vector::Ones(posture_gain_default_.size());
        postureTask_->setMask(posture_mask);
        postureTask_->Kp(posture_gain_default_);
        postureTask_->Kd(2.0*postureTask_->Kp().cwiseSqrt());
    }

    void FrankaHuskyWrapper::mode235ConfigurePostureTask(){
        if (!ismobile_) {
            configureDefaultPostureTask();
            return;
        }

        Vector posture_mask = Vector::Ones(na_-2);
        Vector posture_gain = posture_gain_default_;
        if (posture_gain.size() != na_-2)
            posture_gain = 10000.0*Vector::Ones(na_-2);

        if (mode235_path_straight_path_) {
            // Leader is the transport anchor: keep the CM/lift arm posture
            // stiff enough that follower avoidance does not drag the leader arm
            // into a compensating motion.
            posture_gain << 2200., 1800., 2200., 1800., 1600., 1800., 1600.;
        } else {
            // Follower CM posture is a weak bias, not a hard hold.
            // q1/q3 keep the CM-shaped yaw/null-space posture from collapsing,
            // while q2/q4 stay more compliant so the arm can absorb lateral
            // motion during the follower y-avoidance waypoint.
            posture_gain << 1400., 550., 1400., 550., 800., 900., 700.;
        }

        postureTask_->setMask(posture_mask);
        postureTask_->Kp(posture_gain);
        postureTask_->Kd(2.0*postureTask_->Kp().cwiseSqrt());
    }

    void FrankaHuskyWrapper::ctrl_update(const int& msg){
        ctrl_mode_ = msg;
        ROS_INFO("[ctrltypeCallback] %d", ctrl_mode_);
        mode_change_ = true;
    }

    void FrankaHuskyWrapper::compute(const double& time){
        time_ = time;

        robot_->computeAllTerms(data_, state_.q_, state_.v_);
        // robot_->computeAllTerms_ABA(data_, state_.q_, state_.v_, state_.tau_); //to try to use data.ddq (only computed from ABA) However,the ddq value with ABA is not reasonalbe.

        //---------------------------------------------------------------------------------------------------------//
        //------------------------------------------ basic motion -------------------------------------------------//
        //---------------------------------------------------------------------------------------------------------//
        if (ctrl_mode_ == 1){ //h //init position
            if (mode_change_){
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");
                configureDefaultPostureTask();

                tsid_->addMotionTask(*postureTask_, 1.0, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

                q_ref_.setZero(7);
                q_ref_(0) = 0.0;
                q_ref_(1) = 0.0 * M_PI / 180.0;
                q_ref_(3) = -M_PI / 2.0;
                q_ref_(5) = M_PI / 2.0;
                if (isrobotiq_) q_ref_(6) = -M_PI / 2.0;
                else            q_ref_(6) = -M_PI / 4.0;

                if (ismobile_) trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setInitSample(state_.q_.tail(na_));
                trajPosture_Cubic_->setDuration(3.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);

                reset_control_ = false;
                mode_change_ = false;
            }

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
            if (ismobile_) state_.torque_.head(2).setZero();
        }

        //---------------------------------------------------------------------------------------------------------//
        //----------------------------------------- external EE target servo --------------------------------------//
        //---------------------------------------------------------------------------------------------------------//
        if (ctrl_mode_ == 200){ //external EE target servo (orchestrator approach/lift); mobile holds
            if (mode_change_){
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");
                if (elbow_task_added_) { tsid_->removeTask("task-elbow"); elbow_task_added_ = false; }
                configureDefaultPostureTask();

                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);
                tsid_->addMotionTask(*postureTask_, 3.0, 1);

                if (ismobile_) trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setInitSample(state_.q_.tail(na_));
                trajPosture_Cubic_->setDuration(2.0);
                trajPosture_Cubic_->setStartTime(time_);
                if (ismobile_) trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_));

                eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));

                // seed the external EE target with the current pose until the orchestrator sends one
                if (!has_ee_target_ext_)
                    ee_target_ext_ = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;
                ee_target_dirty_ = true;  // force an initial cubic plan below

                if (ismobile_) {
                    tsid_->addMotionTask(*mobileTask2_, 1.0, 0);
                    mobileTask2_->setDesiredinertia(MatrixXd::Identity(6,6));
                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5);
                    SE3 H_mobile_goal = H_mobile_ref_;
                    if (has_base_target_ext_) {
                        Matrix3d Rz;
                        double yaw = base_target_ext_(2);
                        rotz(yaw, Rz);
                        H_mobile_goal.translation()(0) = base_target_ext_(0);
                        H_mobile_goal.translation()(1) = base_target_ext_(1);
                        H_mobile_goal.rotation() = Rz;
                    }
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(2.0);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_);
                    trajMobile_Cubic_->setGoalSample(H_mobile_goal);
                    base_target_dirty_ = false;
                }

                ee_ref_sample_valid_ = false;
                reset_control_ = false;
                mode_change_ = false;
            }

            // Re-plan a smooth EE cubic whenever the orchestrator pushes a new target.
            // The previous commanded reference is used as the next start sample when available.
            if (ee_target_dirty_){
                if (ee_ref_sample_valid_)
                    vectorToSE3(sampleEE_.pos, H_ee_ref_);
                else
                    H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;
                trajEE_Cubic_->setStartTime(time_);
                trajEE_Cubic_->setDuration(3.0);
                trajEE_Cubic_->setInitSample(H_ee_ref_);
                trajEE_Cubic_->setGoalSample(ee_target_ext_);
                ee_target_dirty_ = false;
            }

            // Re-plan the arm posture toward an externally commanded target.
            if (arm_posture_target_dirty_){
                if (ismobile_) trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
                else           trajPosture_Cubic_->setInitSample(state_.q_.tail(na_));
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setDuration(3.0);
                trajPosture_Cubic_->setGoalSample(arm_posture_target_ext_);
                arm_posture_target_dirty_ = false;
            }

            if (ismobile_ && base_target_dirty_){
                Matrix3d Rz;
                double yaw = base_target_ext_(2);
                rotz(yaw, Rz);
                SE3 H_mobile_goal = robot_->getMobilePosition(data_, 5);
                H_mobile_goal.translation()(0) = base_target_ext_(0);
                H_mobile_goal.translation()(1) = base_target_ext_(1);
                H_mobile_goal.rotation() = Rz;
                H_mobile_ref_ = robot_->getMobilePosition(data_, 5);
                trajMobile_Cubic_->setStartTime(time_);
                trajMobile_Cubic_->setDuration(3.0);
                trajMobile_Cubic_->setInitSample(H_mobile_ref_);
                trajMobile_Cubic_->setGoalSample(H_mobile_goal);
                base_target_dirty_ = false;
            }

            if (ismobile_){
                trajMobile_Cubic_->setCurrentTime(time_);
                sampleMobile_ = trajMobile_Cubic_->computeNext();
                mobileTask2_->setReference(sampleMobile_);
            }

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);

            trajEE_Cubic_->setCurrentTime(time_);
            sampleEE_ = trajEE_Cubic_->computeNext();
            eeTask_->setReference(sampleEE_);
            ee_ref_sample_valid_ = true;

            // 선택 사항: EE null-space에서 elbow 높이만 조정한다.
            if (elbow_z_target_ > 0.0) {
                if (!elbow_task_added_) {
                    tsid_->addMotionTask(*elbowTask_, 2.0, 1);
                    elbow_task_added_ = true;
                }
                pinocchio::SE3 H_elb = robot_->position(data_, robot_->model().getJointId("panda_joint4"));
                H_elb.translation()(2) = elbow_z_target_;  // only z is unmasked
                TrajectorySample s_elb; s_elb.resize(12, 6);
                SE3ToVector(H_elb, s_elb.pos);
                elbowTask_->setReference(s_elb);
            } else if (elbow_task_added_) {
                tsid_->removeTask("task-elbow");
                elbow_task_added_ = false;
            }

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
        }

        if (ctrl_mode_ == 235){ // 최종 협업 운반: base trajectory + EE z/orientation 안정화.
            if (mode_change_){
                const bool leader_x_only_base = false;
                const bool transport_mode = true;
                const bool use_pre_straight =
                    (transport_mode && mode235_pre_straight_x_ > 1e-6);
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");
                if (elbow_task_added_) { tsid_->removeTask("task-elbow"); elbow_task_added_ = false; }
                configureDefaultPostureTask();
                mode235ConfigurePostureTask();

                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                Vector final_ee_gain(6);
                const bool leader_path = mode235_path_straight_path_;
                const double ee_pos_kp =
                    leader_path ? mode235_leader_ee_pos_kp_ : mode235_follower_ee_pos_kp_;
                const double ee_ori_kp =
                    leader_path ? mode235_leader_ee_ori_kp_ : mode235_follower_ee_ori_kp_;
                const double ee_z_kp = std::max(ee_pos_kp, 350.0);
                final_ee_gain << ee_pos_kp, ee_pos_kp, ee_z_kp,
                                 ee_ori_kp, ee_ori_kp, ee_ori_kp;
                Vector ee_mask(6);
                // lateral 회피는 base trajectory가 맡고, EE는 높이와 자세 안정화만 담당한다.
                ee_mask << 0., 0., 1., 1., 1., 1.;
                eeTask_->setMask(ee_mask);
                eeTask_->Kp(final_ee_gain);
                eeTask_->Kd(mode235_follower_ee_kd_ratio_*eeTask_->Kp().cwiseSqrt());
                eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
                const double ee_task_weight = 1.0;
                tsid_->addMotionTask(*eeTask_, ee_task_weight, 1);
                mode235_own_ready_phase_ = -1;
                mode235_peer_ready_phase_ = -1;
                // mode235 priority: 0=bounds, 1=base trajectory + EE 안정화 + posture.
                // leader는 자세 anchor, follower는 EE 안정화와 posture regularization을 함께 쓴다.
                const int mobile_task_priority = 1;
                const int nullspace_priority = 1;
                const int posture_priority = nullspace_priority;
                const double posture_weight = mode235CurrentPostureWeight();
                tsid_->addMotionTask(
                    *postureTask_,
                    posture_weight,
                    posture_priority);
                if (ismobile_) {
                    Vector yaw_mask(3); yaw_mask << 0., 0., 1.;
                    Vector xy_mask(3);
                    xy_mask << 1., (leader_x_only_base ? 0. : 1.), 0.;
                    mobileTask_->setMask(yaw_mask);
                    mobileTask2_->setMask(xy_mask);
                    const double mobile_yaw_kp = mobile_yaw_kp_;
                    const double mobile_yaw_weight = mobile_yaw_weight_;
                    const double mobile_xy_kp = mobile_xy_kp_;
                    mobileTask_->Kp(mobile_yaw_kp*Vector3d::Ones());
                    mobileTask_->Kd(2.5*mobileTask_->Kp().cwiseSqrt());
                    mobileTask2_->Kp(mobile_xy_kp*Vector3d::Ones());
                    mobileTask2_->Kd(2.5*mobileTask2_->Kp().cwiseSqrt());
                    if (!leader_x_only_base)
                        tsid_->addMotionTask(*mobileTask_, mobile_yaw_weight, mobile_task_priority);
                    tsid_->addMotionTask(*mobileTask2_, mobile_xy_weight_, mobile_task_priority);
                    mobileTask_->setDesiredinertia(MatrixXd::Identity(6,6));
                    mobileTask2_->setDesiredinertia(MatrixXd::Identity(6,6));

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5);
                    mode235_path_x_anchor_ = H_mobile_ref_.translation()(0);
                    mode235_path_y_anchor_ = H_mobile_ref_.translation()(1);
                    const pinocchio::SE3 H_ee_init =
                        robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;
                    mode235_mobile_to_ee_offset_ =
                        H_ee_init.translation() - H_mobile_ref_.translation();
                    mode235_ee_rotation_ref_ = H_ee_init.rotation();
                    // mode235 주행 중 높이는 lift 완료 직후의 양팔 평균 EE 높이를 기준으로 고정한다.
                    mode235_ee_z_ref_ = H_ee_init.translation()(2);
                    if (mode235_has_lift_height_metrics_) {
                        mode235_ee_z_ref_ =
                            0.5 * (mode235_latest_leader_ee_z_ + mode235_latest_follower_ee_z_);
                    }
                    sampleEE_.vel.setZero();
                    sampleEE_.acc.setZero();
                    SE3ToVector(H_ee_init, sampleEE_.pos);
                    eeTask_->setReference(sampleEE_);
                    const auto mode235_pathX = [&](double x) {
                        return mode235_path_relative_x_ ? mode235_path_x_anchor_ + x : x;
                    };
                    const auto mode235_pathY = [&](double y) {
                        return mode235_path_relative_y_ ? mode235_path_y_anchor_ + y : y;
                    };
                    const auto phaseTheta = [&](double theta_deg) {
                        return theta_deg;
                    };
                    const auto phaseY = [&](double y) {
                        return y;
                    };
                    const auto straightDuration = [&](double straight_duration, double default_duration) {
                        return (mode235_path_straight_path_ && straight_duration > 1e-6)
                                   ? straight_duration
                                   : default_duration;
                    };
                    const double first_x =
                        use_pre_straight ? mode235_pre_straight_x_
                                         : (mode235_path_straight_path_ ? mode235_path_straight_x_scale_ * mode235_path_p1_x_
                                                                   : mode235_path_p1_x_);
                    const double first_y =
                        use_pre_straight ? 0.0 : phaseY(mode235_path_straight_path_ ? mode235_path_straight_y_ : mode235_path_p1_y_);
                    const double first_theta =
                        use_pre_straight ? 0.0 : phaseTheta(mode235_path_straight_path_ ? mode235_path_straight_theta_ : mode235_path_p1_theta_);
                    const double first_duration =
                        use_pre_straight ? mode235_pre_straight_duration_
                                         : straightDuration(mode235_path_straight_p1_duration_, mode235_path_p1_duration_);
                    SE3 H_goal = H_mobile_ref_;
                    H_goal.translation()(0) = mode235_pathX(first_x);
                    H_goal.translation()(1) = mode235_pathY(first_y);
                    H_goal.rotation() =
                        Eigen::AngleAxisd(first_theta * M_PI / 180.0,
                                          Eigen::Vector3d::UnitZ()).toRotationMatrix();
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(std::max(0.1, first_duration));
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_);
                    trajMobile_Cubic_->setGoalSample(H_goal);
                }

                const Vector mode235_path_posture_now = ismobile_ ? state_.q_.tail(na_-2) : state_.q_.tail(na_);
                Vector mode235_path_posture_goal = transport_mode ? mode235_path_posture_now : q_ref_;
                trajPosture_Cubic_->setInitSample(mode235_path_posture_now);
                trajPosture_Cubic_->setGoalSample(mode235_path_posture_goal);
                trajPosture_Cubic_->setDuration(3.0);
                trajPosture_Cubic_->setStartTime(time_);

                mode235_path_phase_ = use_pre_straight ? 0 : 1;
                mode235_path_done_phase_ = -1;
                mode235_path_phase_start_time_ = time_;
                reset_control_ = false;
                mode_change_ = false;
                const double log_x_offset = use_pre_straight ? mode235_pre_straight_x_ : 0.0;
                const auto logTheta = [&](double theta_deg) {
                    return theta_deg;
                };
                const auto logY = [&](double y) {
                    return y;
                };
                const auto logStraightDuration = [&](double straight_duration, double default_duration) {
                    return (mode235_path_straight_path_ && straight_duration > 1e-6)
                               ? straight_duration
                               : default_duration;
                };
                const double log_p1_duration = logStraightDuration(mode235_path_straight_p1_duration_, mode235_path_p1_duration_);
                const double log_p2_duration = logStraightDuration(mode235_path_straight_p2_duration_, mode235_path_p2_duration_);
                const double log_p3_duration = logStraightDuration(mode235_path_straight_p3_duration_, mode235_path_p3_duration_);
                const double log_p4_duration = logStraightDuration(mode235_path_straight_p4_duration_, mode235_path_p4_duration_);
                ROS_INFO("[mode%d] %s role=%s pre_straight=%s ee_se3_offset=true p0=(%.2f,0,0,%.1fs) p1=(%.2f,%.2f,%.0f,%.1fs) p2=(%.2f,%.2f,%.0f,%.1fs) p3=(%.2f,%.2f,%.0f,%.1fs) p4=(%.2f,%.2f,%.0f,%.1fs) p5=(%.2f,%.2f,%.0f,%.1fs)",
                         ctrl_mode_,
                         robot_node_.c_str(),
                         mode235_path_straight_path_ ? "leader" : "follower",
                         use_pre_straight ? "true" : "false",
                         mode235_pre_straight_x_, mode235_pre_straight_duration_,
                         log_x_offset + mode235_path_p1_x_, logY(mode235_path_straight_path_ ? mode235_path_straight_y_ : mode235_path_p1_y_), logTheta(mode235_path_straight_path_ ? mode235_path_straight_theta_ : mode235_path_p1_theta_), log_p1_duration,
                         log_x_offset + mode235_path_p2_x_, logY(mode235_path_straight_path_ ? mode235_path_straight_y_ : mode235_path_p2_y_), logTheta(mode235_path_straight_path_ ? mode235_path_straight_theta_ : mode235_path_p2_theta_), log_p2_duration,
                         log_x_offset + mode235_path_p3_x_, logY(mode235_path_straight_path_ ? mode235_path_straight_y_ : mode235_path_p3_y_), logTheta(mode235_path_straight_path_ ? mode235_path_straight_theta_ : mode235_path_p3_theta_), log_p3_duration,
                         log_x_offset + mode235_path_p4_x_, logY(mode235_path_straight_path_ ? mode235_path_straight_y_ : mode235_path_p4_y_), logTheta(mode235_path_straight_path_ ? mode235_path_straight_theta_ : mode235_path_p4_theta_), log_p4_duration,
                         log_x_offset + mode235_path_p5_x_, logY(mode235_path_straight_path_ ? mode235_path_straight_y_ : mode235_path_p5_y_), logTheta(mode235_path_straight_path_ ? mode235_path_straight_theta_ : mode235_path_p5_theta_), mode235_path_p5_duration_);
	                if (transport_mode) {
	                    const std::string arm_status =
	                        mode235_path_straight_path_
                            ? "leader mobile path + EE z/orientation stabilizer"
                            : "follower waypoint path + EE z/orientation stabilizer";
                    ROS_INFO("[mode%d] %s arm=%s mobile_priority=%d ee_priority=%d posture_priority=%d posture_weight=%.3f",
                             ctrl_mode_, robot_node_.c_str(),
		                             arm_status.c_str(),
		                             mobile_task_priority,
		                             1,
	                             posture_priority,
	                             posture_weight);
                }
            }

            const bool transport_mode_active = true;
            const auto mode235_pathXCurrent = [&](double x) {
                return mode235_path_relative_x_ ? mode235_path_x_anchor_ + x : x;
            };
            const auto mode235_pathYCurrent = [&](double y) {
                return mode235_path_relative_y_ ? mode235_path_y_anchor_ + y : y;
            };
            const auto phaseThetaCurrent = [&](double theta_deg) {
                return theta_deg;
            };
            const auto phaseYCurrent = [&](double y) {
                return y;
            };
            const bool mode235_use_final_straight =
                (transport_mode_active && mode235_path_p5_x_ > mode235_path_p4_x_ + 1e-6);
            const auto mode235_pathPhaseRelativeX = [&](int phase) {
                const double x_offset = transport_mode_active ? mode235_pre_straight_x_ : 0.0;
                if (phase == 0) return mode235_pre_straight_x_;
                double x = mode235_path_p5_x_;
                if (phase == 1) x = mode235_path_p1_x_;
                else if (phase == 2) x = mode235_path_p2_x_;
                else if (phase == 3) x = mode235_path_p3_x_;
                else if (phase == 4 || !mode235_use_final_straight) x = mode235_path_p4_x_;
                if (mode235_path_straight_path_)
                    x *= mode235_path_straight_x_scale_;
                return x_offset + x;
            };
            const auto mode235_pathTargetForPhase = [&](int phase,
                                                   double & x,
                                                   double & y,
                                                   double & theta_deg,
                                                   double & duration) {
                if (phase == 0) {
                    x = mode235_pathXCurrent(mode235_pre_straight_x_);
                    y = mode235_pathYCurrent(0.0);
                    theta_deg = 0.0;
                    duration = mode235_pre_straight_duration_;
                } else if (phase == 1) {
                    x = mode235_pathXCurrent(mode235_pathPhaseRelativeX(1));
                    y = mode235_pathYCurrent(phaseYCurrent(mode235_path_straight_path_ ? mode235_path_straight_y_ : mode235_path_p1_y_));
                    theta_deg = phaseThetaCurrent(mode235_path_straight_path_ ? mode235_path_straight_theta_ : mode235_path_p1_theta_);
                    duration = (mode235_path_straight_path_ && mode235_path_straight_p1_duration_ > 1e-6)
                                   ? mode235_path_straight_p1_duration_
                                   : mode235_path_p1_duration_;
                } else if (phase == 2) {
                    x = mode235_pathXCurrent(mode235_pathPhaseRelativeX(2));
                    y = mode235_pathYCurrent(phaseYCurrent(mode235_path_straight_path_ ? mode235_path_straight_y_ : mode235_path_p2_y_));
                    theta_deg = phaseThetaCurrent(mode235_path_straight_path_ ? mode235_path_straight_theta_ : mode235_path_p2_theta_);
                    duration = (mode235_path_straight_path_ && mode235_path_straight_p2_duration_ > 1e-6)
                                   ? mode235_path_straight_p2_duration_
                                   : mode235_path_p2_duration_;
                } else if (phase == 3) {
                    x = mode235_pathXCurrent(mode235_pathPhaseRelativeX(3));
                    y = mode235_pathYCurrent(phaseYCurrent(mode235_path_straight_path_ ? mode235_path_straight_y_ : mode235_path_p3_y_));
                    theta_deg = phaseThetaCurrent(mode235_path_straight_path_ ? mode235_path_straight_theta_ : mode235_path_p3_theta_);
                    duration = (mode235_path_straight_path_ && mode235_path_straight_p3_duration_ > 1e-6)
                                   ? mode235_path_straight_p3_duration_
                                   : mode235_path_p3_duration_;
                } else if (phase == 4 || !mode235_use_final_straight) {
                    x = mode235_pathXCurrent(mode235_pathPhaseRelativeX(4));
                    y = mode235_pathYCurrent(phaseYCurrent(mode235_path_straight_path_ ? mode235_path_straight_y_ : mode235_path_p4_y_));
                    theta_deg = phaseThetaCurrent(mode235_path_straight_path_ ? mode235_path_straight_theta_ : mode235_path_p4_theta_);
                    duration = (mode235_path_straight_path_ && mode235_path_straight_p4_duration_ > 1e-6)
                                   ? mode235_path_straight_p4_duration_
                                   : mode235_path_p4_duration_;
                } else {
                    x = mode235_pathXCurrent(mode235_pathPhaseRelativeX(5));
                    y = mode235_pathYCurrent(phaseYCurrent(mode235_path_straight_path_ ? mode235_path_straight_y_ : mode235_path_p5_y_));
                    theta_deg = phaseThetaCurrent(mode235_path_straight_path_ ? mode235_path_straight_theta_ : mode235_path_p5_theta_);
                    duration = mode235_path_p5_duration_;
                }
            };
            const auto mode235_pathPhaseReady = [&](int phase) {
                if (!transport_mode_active)
                    return true;
                double tx = 0.0, ty = 0.0, theta_deg = 0.0, duration = 0.0;
                mode235_pathTargetForPhase(phase, tx, ty, theta_deg, duration);
                const SE3 H_actual = robot_->getMobilePosition(data_, 5);
                const double actual_yaw =
                    atan2(-H_actual.rotation()(0, 1), H_actual.rotation()(0, 0));
                const double target_yaw = theta_deg * M_PI / 180.0;
                double yaw_err = target_yaw - actual_yaw;
                while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
                while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
                const double dx = tx - H_actual.translation()(0);
                const double dy = ty - H_actual.translation()(1);
                const double xy_err = std::sqrt(dx * dx + dy * dy);
                const double yaw_err_deg = std::abs(yaw_err) * 180.0 / M_PI;
                const double xy_tolerance =
                    (phase == 5) ? std::max(mode235_phase_xy_tolerance_, 0.55)
                                 : ((phase == 3) ? mode235_phase3_xy_tolerance_
                                                 : mode235_phase_xy_tolerance_);
                const bool ready =
                    xy_err <= xy_tolerance &&
                    yaw_err_deg <= mode235_phase_yaw_tolerance_deg_;
                if (ready) {
                    mode235_own_ready_phase_ = std::max(mode235_own_ready_phase_, phase);
                    std_msgs::Int16 ready_msg;
                    ready_msg.data = static_cast<int16_t>(phase);
                    mode235_phase_ready_pub_.publish(ready_msg);
                }
                const bool peer_ready_topic = mode235_peer_ready_phase_ >= phase;
                const bool peer_ready = peer_ready_topic;
                if (!ready || !peer_ready) {
                    ROS_INFO_THROTTLE(
                        2.0,
                        "[mode%d wait] %s phase=%d self=%s peer=%s target=(%.3f, %.3f, %.1fdeg) actual=(%.3f, %.3f, %.1fdeg) err=(xy %.3f, yaw %.1fdeg) tol=(%.3f, %.1fdeg)",
                        ctrl_mode_, robot_node_.c_str(), phase,
                        ready ? "ready" : "wait",
                        peer_ready_topic ? "ready" : "wait",
                        tx, ty, theta_deg,
                        H_actual.translation()(0), H_actual.translation()(1), actual_yaw * 180.0 / M_PI,
                        xy_err, yaw_err_deg,
                        xy_tolerance, mode235_phase_yaw_tolerance_deg_);
	                }
	                return ready && peer_ready;
	            };
            const auto mode235_pathDurationForPhase = [&](int phase) {
                if (!mode235_path_straight_path_)
                    return phase == 1 ? mode235_path_p1_duration_
                         : phase == 2 ? mode235_path_p2_duration_
                         : phase == 3 ? mode235_path_p3_duration_
                         : phase == 4 ? mode235_path_p4_duration_
                                      : mode235_path_p5_duration_;
                if (phase == 1 && mode235_path_straight_p1_duration_ > 1e-6) return mode235_path_straight_p1_duration_;
                if (phase == 2 && mode235_path_straight_p2_duration_ > 1e-6) return mode235_path_straight_p2_duration_;
                if (phase == 3 && mode235_path_straight_p3_duration_ > 1e-6) return mode235_path_straight_p3_duration_;
                if (phase == 4 && mode235_path_straight_p4_duration_ > 1e-6) return mode235_path_straight_p4_duration_;
                return phase == 1 ? mode235_path_p1_duration_
                     : phase == 2 ? mode235_path_p2_duration_
                     : phase == 3 ? mode235_path_p3_duration_
                     : phase == 4 ? mode235_path_p4_duration_
                                  : mode235_path_p5_duration_;
            };

	            if (ismobile_ && mode235_path_phase_ == 0 &&
	                (time_ - mode235_path_phase_start_time_) >= std::max(0.1, mode235_pre_straight_duration_) &&
                mode235_pathPhaseReady(0)) {
                const SE3 H_cur = robot_->getMobilePosition(data_, 5);
                const auto mode235_pathX = [&](double x) {
                    return mode235_path_relative_x_ ? mode235_path_x_anchor_ + x : x;
                };
                const auto mode235_pathY = [&](double y) {
                    return mode235_path_relative_y_ ? mode235_path_y_anchor_ + y : y;
                };
                const double p1_y = phaseYCurrent(mode235_path_straight_path_ ? mode235_path_straight_y_ : mode235_path_p1_y_);
                const double p1_theta = phaseThetaCurrent(mode235_path_straight_path_ ? mode235_path_straight_theta_ : mode235_path_p1_theta_);
                const double p1_x = mode235_pathPhaseRelativeX(1);
                SE3 H_goal = H_cur;
                H_goal.translation()(0) = mode235_pathX(p1_x);
                H_goal.translation()(1) = mode235_pathY(p1_y);
                H_goal.rotation() =
                    Eigen::AngleAxisd(p1_theta * M_PI / 180.0,
                                      Eigen::Vector3d::UnitZ()).toRotationMatrix();
		                trajMobile_Cubic_->setStartTime(time_);
		                trajMobile_Cubic_->setDuration(std::max(0.1, mode235_pathDurationForPhase(1)));
		                trajMobile_Cubic_->setInitSample(H_cur);
		                trajMobile_Cubic_->setGoalSample(H_goal);
                mode235_path_phase_ = 1;
                mode235_path_phase_start_time_ = time_;
                ROS_INFO("[mode%d] %s phase1 avoid-edge start from joint1_global=(%.3f, %.3f) target=(%.2f, %.2f, %.0fdeg)",
                         ctrl_mode_, robot_node_.c_str(), H_cur.translation()(0), H_cur.translation()(1),
                         mode235_pathX(p1_x), mode235_pathY(p1_y), p1_theta);
            }

		            if (ismobile_ && mode235_path_phase_ == 1 &&
		                (time_ - mode235_path_phase_start_time_) >= std::max(0.1, mode235_pathDurationForPhase(1)) &&
                mode235_pathPhaseReady(1)) {
                const SE3 H_cur = robot_->getMobilePosition(data_, 5);
                const double actual_yaw =
                    atan2(-H_cur.rotation()(0, 1), H_cur.rotation()(0, 0));
                ROS_INFO("[mode%d reached] %s phase1 actual=(%.3f, %.3f, %.1fdeg)",
                         ctrl_mode_, robot_node_.c_str(),
                         H_cur.translation()(0), H_cur.translation()(1),
                         actual_yaw * 180.0 / M_PI);
                const auto mode235_pathX = [&](double x) {
                    return mode235_path_relative_x_ ? mode235_path_x_anchor_ + x : x;
                };
                const auto mode235_pathY = [&](double y) {
                    return mode235_path_relative_y_ ? mode235_path_y_anchor_ + y : y;
                };
                const double p2_y = phaseYCurrent(mode235_path_straight_path_ ? mode235_path_straight_y_ : mode235_path_p2_y_);
                const double p2_theta = phaseThetaCurrent(mode235_path_straight_path_ ? mode235_path_straight_theta_ : mode235_path_p2_theta_);
                const double p2_x = mode235_pathPhaseRelativeX(2);
                SE3 H_goal = H_cur;
                H_goal.translation()(0) = mode235_pathX(p2_x);
                H_goal.translation()(1) = mode235_pathY(p2_y);
                H_goal.rotation() =
                    Eigen::AngleAxisd(p2_theta * M_PI / 180.0,
                                      Eigen::Vector3d::UnitZ()).toRotationMatrix();
	                trajMobile_Cubic_->setStartTime(time_);
	                trajMobile_Cubic_->setDuration(std::max(0.1, mode235_pathDurationForPhase(2)));
	                trajMobile_Cubic_->setInitSample(H_cur);
	                trajMobile_Cubic_->setGoalSample(H_goal);
	                mode235_path_phase_ = 2;
                mode235_path_phase_start_time_ = time_;
                ROS_INFO("[mode%d] %s phase2 settle start from joint1_global=(%.3f, %.3f) target=(%.2f, %.2f, %.0fdeg)",
                         ctrl_mode_, robot_node_.c_str(), H_cur.translation()(0), H_cur.translation()(1),
                         mode235_pathX(p2_x), mode235_pathY(p2_y), p2_theta);
            }

		            if (ismobile_ && mode235_path_phase_ == 2 &&
		                (time_ - mode235_path_phase_start_time_) >= std::max(0.1, mode235_pathDurationForPhase(2)) &&
                mode235_pathPhaseReady(2)) {
                const SE3 H_cur = robot_->getMobilePosition(data_, 5);
                const double actual_yaw =
                    atan2(-H_cur.rotation()(0, 1), H_cur.rotation()(0, 0));
                ROS_INFO("[mode%d reached] %s phase2 actual=(%.3f, %.3f, %.1fdeg)",
                         ctrl_mode_, robot_node_.c_str(),
                         H_cur.translation()(0), H_cur.translation()(1),
                         actual_yaw * 180.0 / M_PI);
                const auto mode235_pathX = [&](double x) {
                    return mode235_path_relative_x_ ? mode235_path_x_anchor_ + x : x;
                };
                const auto mode235_pathY = [&](double y) {
                    return mode235_path_relative_y_ ? mode235_path_y_anchor_ + y : y;
                };
                const double p3_y = phaseYCurrent(mode235_path_straight_path_ ? mode235_path_straight_y_ : mode235_path_p3_y_);
                const double p3_theta = phaseThetaCurrent(mode235_path_straight_path_ ? mode235_path_straight_theta_ : mode235_path_p3_theta_);
                const double p3_x = mode235_pathPhaseRelativeX(3);
                SE3 H_goal = H_cur;
                H_goal.translation()(0) = mode235_pathX(p3_x);
                H_goal.translation()(1) = mode235_pathY(p3_y);
                H_goal.rotation() =
                    Eigen::AngleAxisd(p3_theta * M_PI / 180.0,
                                      Eigen::Vector3d::UnitZ()).toRotationMatrix();
                trajMobile_Cubic_->setStartTime(time_);
                trajMobile_Cubic_->setDuration(std::max(0.1, mode235_pathDurationForPhase(3)));
                trajMobile_Cubic_->setInitSample(H_cur);
                trajMobile_Cubic_->setGoalSample(H_goal);
                mode235_path_phase_ = 3;
                mode235_path_phase_start_time_ = time_;
                ROS_INFO("[mode%d] %s phase3 return-edge start from joint1_global=(%.3f, %.3f) target=(%.2f, %.2f, %.0fdeg)",
                         ctrl_mode_, robot_node_.c_str(), H_cur.translation()(0), H_cur.translation()(1),
                         mode235_pathX(p3_x), mode235_pathY(p3_y), p3_theta);
            }

		            if (ismobile_ && mode235_path_phase_ == 3 &&
		                (time_ - mode235_path_phase_start_time_) >= std::max(0.1, mode235_pathDurationForPhase(3)) &&
                mode235_pathPhaseReady(3)) {
                const SE3 H_cur = robot_->getMobilePosition(data_, 5);
                const double actual_yaw =
                    atan2(-H_cur.rotation()(0, 1), H_cur.rotation()(0, 0));
                ROS_INFO("[mode%d reached] %s phase3 actual=(%.3f, %.3f, %.1fdeg)",
                         ctrl_mode_, robot_node_.c_str(),
                         H_cur.translation()(0), H_cur.translation()(1),
                         actual_yaw * 180.0 / M_PI);
                const auto mode235_pathX = [&](double x) {
                    return mode235_path_relative_x_ ? mode235_path_x_anchor_ + x : x;
                };
                const auto mode235_pathY = [&](double y) {
                    return mode235_path_relative_y_ ? mode235_path_y_anchor_ + y : y;
                };
                const double p4_y = phaseYCurrent(mode235_path_straight_path_ ? mode235_path_straight_y_ : mode235_path_p4_y_);
                const double p4_theta = phaseThetaCurrent(mode235_path_straight_path_ ? mode235_path_straight_theta_ : mode235_path_p4_theta_);
                const double p4_x = mode235_pathPhaseRelativeX(4);
                SE3 H_goal = H_cur;
                H_goal.translation()(0) = mode235_pathX(p4_x);
                H_goal.translation()(1) = mode235_pathY(p4_y);
                H_goal.rotation() =
                    Eigen::AngleAxisd(p4_theta * M_PI / 180.0,
                                      Eigen::Vector3d::UnitZ()).toRotationMatrix();
	                trajMobile_Cubic_->setStartTime(time_);
	                trajMobile_Cubic_->setDuration(std::max(0.1, mode235_pathDurationForPhase(4)));
	                trajMobile_Cubic_->setInitSample(H_cur);
	                trajMobile_Cubic_->setGoalSample(H_goal);
	                mode235_path_phase_ = 4;
                mode235_path_phase_start_time_ = time_;
                ROS_INFO("[mode%d] %s phase4 final-settle start from joint1_global=(%.3f, %.3f) target=(%.2f, %.2f, %.0fdeg)",
                         ctrl_mode_, robot_node_.c_str(), H_cur.translation()(0), H_cur.translation()(1),
                         mode235_pathX(p4_x), mode235_pathY(p4_y), p4_theta);
            }

		            if (ismobile_ && mode235_path_phase_ == 4 && mode235_use_final_straight &&
		                (time_ - mode235_path_phase_start_time_) >= std::max(0.1, mode235_pathDurationForPhase(4)) &&
                mode235_pathPhaseReady(4)) {
                const SE3 H_cur = robot_->getMobilePosition(data_, 5);
                const double actual_yaw =
                    atan2(-H_cur.rotation()(0, 1), H_cur.rotation()(0, 0));
                ROS_INFO("[mode%d reached] %s phase4 actual=(%.3f, %.3f, %.1fdeg)",
                         ctrl_mode_, robot_node_.c_str(),
                         H_cur.translation()(0), H_cur.translation()(1),
                         actual_yaw * 180.0 / M_PI);
                const auto mode235_pathX = [&](double x) {
                    return mode235_path_relative_x_ ? mode235_path_x_anchor_ + x : x;
                };
                const auto mode235_pathY = [&](double y) {
                    return mode235_path_relative_y_ ? mode235_path_y_anchor_ + y : y;
                };
                const double p5_y = phaseYCurrent(mode235_path_straight_path_ ? mode235_path_straight_y_ : mode235_path_p5_y_);
                const double p5_theta = phaseThetaCurrent(mode235_path_straight_path_ ? mode235_path_straight_theta_ : mode235_path_p5_theta_);
                const double p5_x = mode235_pathPhaseRelativeX(5);
                SE3 H_goal = H_cur;
                H_goal.translation()(0) = mode235_pathX(p5_x);
                H_goal.translation()(1) = mode235_pathY(p5_y);
                H_goal.rotation() =
                    Eigen::AngleAxisd(p5_theta * M_PI / 180.0,
                                      Eigen::Vector3d::UnitZ()).toRotationMatrix();
                trajMobile_Cubic_->setStartTime(time_);
                trajMobile_Cubic_->setDuration(std::max(0.1, mode235_path_p5_duration_));
                trajMobile_Cubic_->setInitSample(H_cur);
                trajMobile_Cubic_->setGoalSample(H_goal);
                mode235_path_phase_ = 5;
                mode235_path_phase_start_time_ = time_;
                ROS_INFO("[mode%d] %s phase5 final-straight start from joint1_global=(%.3f, %.3f) target=(%.2f, %.2f, %.0fdeg)",
                         ctrl_mode_, robot_node_.c_str(), H_cur.translation()(0), H_cur.translation()(1),
                         mode235_pathX(p5_x), mode235_pathY(p5_y), p5_theta);
            }

            if (ismobile_){
                trajMobile_Cubic_->setCurrentTime(time_);
                sampleMobile_ = trajMobile_Cubic_->computeNext();
                mobileTask_->setReference(sampleMobile_);
                mobileTask2_->setReference(sampleMobile_);

                pinocchio::SE3 H_ref_now;
                vectorToSE3(sampleMobile_.pos, H_ref_now);
                H_ee_ref_ = H_ref_now;
                H_ee_ref_.translation() =
                    H_ref_now.translation() + mode235_mobile_to_ee_offset_;
                H_ee_ref_.translation()(2) = mode235_ee_z_ref_;
                H_ee_ref_.rotation() = mode235_ee_rotation_ref_;
                SE3ToVector(H_ee_ref_, sampleEE_.pos);
                sampleEE_.vel.setZero();
                sampleEE_.acc.setZero();
                sampleEE_.vel.head(3) = sampleMobile_.vel.head(3);
                sampleEE_.acc.head(3) = sampleMobile_.acc.head(3);
                sampleEE_.vel(2) = 0.0;
                sampleEE_.acc(2) = 0.0;
                eeTask_->setReference(sampleEE_);
                const pinocchio::SE3 H_actual = robot_->getMobilePosition(data_, 5);
                const double ref_yaw =
                    atan2(-H_ref_now.rotation()(0, 1), H_ref_now.rotation()(0, 0));
                const double actual_yaw =
                    atan2(-H_actual.rotation()(0, 1), H_actual.rotation()(0, 0));
                double yaw_err = ref_yaw - actual_yaw;
                while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
                while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
                ROS_DEBUG_THROTTLE(5.0,
                    "[mode%d] %s phase=%d joint1_global actual=(%.3f, %.3f, %.1fdeg) ref=(%.3f, %.3f, %.1fdeg) err=(%.3f, %.3f, %.1fdeg)",
                    ctrl_mode_, robot_node_.c_str(), mode235_path_phase_,
                    H_actual.translation()(0), H_actual.translation()(1), actual_yaw * 180.0 / M_PI,
                    H_ref_now.translation()(0), H_ref_now.translation()(1), ref_yaw * 180.0 / M_PI,
                    H_ref_now.translation()(0) - H_actual.translation()(0),
                    H_ref_now.translation()(1) - H_actual.translation()(1),
                    yaw_err * 180.0 / M_PI);

                if (mode235_path_phase_ == 4 && mode235_path_done_phase_ < 4 && !mode235_use_final_straight &&
                    (time_ - mode235_path_phase_start_time_) >= std::max(0.1, mode235_pathDurationForPhase(4)) &&
                    mode235_pathPhaseReady(4)) {
                    const double p4_x_rel = mode235_pathPhaseRelativeX(4);
                    const double p4_x_target = mode235_path_relative_x_ ? mode235_path_x_anchor_ + p4_x_rel : p4_x_rel;
                    const double p4_y_input = phaseYCurrent(mode235_path_straight_path_ ? mode235_path_straight_y_ : mode235_path_p4_y_);
                    const double p4_y_target = mode235_path_relative_y_ ? mode235_path_y_anchor_ + p4_y_input : p4_y_input;
                    const double p4_theta_target = phaseThetaCurrent(mode235_path_straight_path_ ? mode235_path_straight_theta_ : mode235_path_p4_theta_);
                    mode235_path_done_phase_ = 4;
                    ROS_INFO(
                        "[mode%d done] %s joint1_global actual=(%.3f, %.3f, %.1fdeg) target=(%.3f, %.3f, %.1fdeg) err=(%.3f, %.3f, %.1fdeg)",
                        ctrl_mode_, robot_node_.c_str(),
                        H_actual.translation()(0), H_actual.translation()(1), actual_yaw * 180.0 / M_PI,
                        p4_x_target, p4_y_target, p4_theta_target,
                        p4_x_target - H_actual.translation()(0),
                        p4_y_target - H_actual.translation()(1),
                        yaw_err * 180.0 / M_PI);
                }
                if (mode235_path_phase_ == 5 && mode235_path_done_phase_ < 5 &&
                    (time_ - mode235_path_phase_start_time_) >= std::max(0.1, mode235_path_p5_duration_) &&
                    mode235_pathPhaseReady(5)) {
                    const double p5_x_rel = mode235_pathPhaseRelativeX(5);
                    const double p5_x_target = mode235_path_relative_x_ ? mode235_path_x_anchor_ + p5_x_rel : p5_x_rel;
                    const double p5_y_input = phaseYCurrent(mode235_path_straight_path_ ? mode235_path_straight_y_ : mode235_path_p5_y_);
                    const double p5_y_target = mode235_path_relative_y_ ? mode235_path_y_anchor_ + p5_y_input : p5_y_input;
                    const double p5_theta_target = phaseThetaCurrent(mode235_path_straight_path_ ? mode235_path_straight_theta_ : mode235_path_p5_theta_);
                    mode235_path_done_phase_ = 5;
                    ROS_INFO(
                        "[mode%d done] %s joint1_global actual=(%.3f, %.3f, %.1fdeg) target=(%.3f, %.3f, %.1fdeg) err=(%.3f, %.3f, %.1fdeg)",
                        ctrl_mode_, robot_node_.c_str(),
                        H_actual.translation()(0), H_actual.translation()(1), actual_yaw * 180.0 / M_PI,
                        p5_x_target, p5_y_target, p5_theta_target,
                        p5_x_target - H_actual.translation()(0),
                        p5_y_target - H_actual.translation()(1),
                        yaw_err * 180.0 / M_PI);
                }
            }

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
        }

        // cout << state_.torque_.transpose() << endl;
    }

    // ---- 협업 운반 orchestration node의 외부 task/posture reference ----
    void FrankaHuskyWrapper::set_ee_target(const pinocchio::SE3 & target){
        ee_target_ext_ = target;
        has_ee_target_ext_ = true;
        ee_target_dirty_ = true;   // mode 200 cubic reference 갱신
    }
    void FrankaHuskyWrapper::set_base_target(double x, double y, double yaw){
        const Vector3d requested(x, y, yaw);
        // CM runtime repeats targets for delivery robustness. Do not restart
        // the three-second base cubic on every identical message.
        if (has_base_target_ext_ && (base_target_ext_ - requested).norm() < 1e-6)
            return;
        base_target_ext_ = requested;
        has_base_target_ext_ = true;
        base_target_dirty_ = true; // base cubic reference 갱신
    }
    void FrankaHuskyWrapper::set_arm_posture_target(const Eigen::VectorXd & q_arm){
        // 같은 posture reference가 반복 publish되어 cubic이 계속 재시작되는 것을 막는다.
        if (has_arm_posture_target_ext_ &&
            arm_posture_target_ext_.size() == q_arm.size() &&
            (arm_posture_target_ext_ - q_arm).norm() < 1e-6)
            return;
        arm_posture_target_ext_ = q_arm;
        has_arm_posture_target_ext_ = true;
        arm_posture_target_dirty_ = true;  // posture cubic reference 갱신
    }

    void FrankaHuskyWrapper::franka_output(VectorXd & qacc) { //from here to main code
        if (ismobile_) qacc = state_.torque_.tail(na_-2);
        else qacc = state_.torque_.tail(na_);
    }
    void FrankaHuskyWrapper::husky_output(VectorXd & qvel) {
        qvel = state_.torque_.head(2);

        const double cmd_limit = 50.0;
        for (int i = 0; i < qvel.size(); ++i) {
            if (qvel(i) > cmd_limit) qvel(i) = cmd_limit;
            if (qvel(i) < -cmd_limit) qvel(i) = -cmd_limit;
        }
    }

    void FrankaHuskyWrapper::com(Eigen::Vector3d & com){
        //API:Vector of subtree center of mass positions expressed in the root joint of the subtree.
        //API:In other words, com[j] is the CoM position of the subtree supported by joint j and expressed in the joint frame .
        //API:The element com[0] corresponds to the center of mass position of the whole model and expressed in the global frame.
        com = robot_->com(data_);
    }

    void FrankaHuskyWrapper::position(pinocchio::SE3 & oMi){
        //API:Vector of absolute joint placements (wrt the world).
        oMi = robot_->position(data_, robot_->model().getJointId("panda_joint7"));
    }
    void FrankaHuskyWrapper::elbow_position(pinocchio::SE3 & oMi){
        oMi = robot_->position(data_, robot_->model().getJointId("panda_joint4"));
    }

    void FrankaHuskyWrapper::position_offset(pinocchio::SE3 & oMi){
        //API:Vector of absolute joint placements (wrt the world).
        oMi = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;
    }

    void FrankaHuskyWrapper::position_link0_offset(pinocchio::SE3 & oMi){
        //API:Vector of absolute joint placements (wrt the world).
        SE3 panda_link0_ee;
        panda_link0_ee = robot_->framePosition(data_, robot_->model().getFrameId("panda_link0"));

        SE3 panda_joint7_offset;
        this->position_offset(panda_joint7_offset);

        oMi = panda_link0_ee.inverse() * panda_joint7_offset;
    }

    void FrankaHuskyWrapper::velocity(pinocchio::Motion & vel){
        //API:Vector of joint velocities expressed at the centers of the joints.
        vel = robot_->velocity(data_, robot_->model().getJointId("panda_joint7"));

        // NOTES ////////////////////////////////////////////////////////////////////////////
        // data.v = f.placement.actInv(data.v[f.parent]), both are in LOCAL coordinate
        // T_offset.act(v_frame) is WRONG method, It means that applying offset w.r.t. global coord. to EE frame.
        //////////////////////////////////////////////////////////////////////////////////////

        // code comparison ///////////////////////////
        // To apply offset, Adj_mat should be used
        //////////////////////////////////////////////
        // cout << "data.v" << endl;
        // cout <<  robot_->velocity(data_, robot_->model().getJointId("panda_joint7")) << endl;

        // cout << "data.v with Adj_mat_" << endl;
        // cout << vel.linear() + Adj_mat_.topRightCorner(3,3) * vel.angular() << endl;
        // cout << vel.angular() << endl;
    }

    void FrankaHuskyWrapper::velocity_offset(pinocchio::Motion & vel){
        //API:Vector of joint velocities expressed at the centers of the joints.
        vel = robot_->velocity(data_, robot_->model().getJointId("panda_joint7"));

        vel.linear() = vel.linear() + Adj_mat_.topRightCorner(3,3) * vel.angular();
        vel.angular() = vel.angular();
    }

    void FrankaHuskyWrapper::velocity_origin(pinocchio::Motion & vel){
        //API:Vector of joint velocities expressed at the origin. (data.ov)
        //Same with "vel = m_wMl.act(v_frame);"
        vel = robot_->velocity_origin(data_, robot_->model().getJointId("panda_joint7"));
    }

    void FrankaHuskyWrapper::acceleration(pinocchio::Motion & accel){
        //API:Vector of joint accelerations expressed at the centers of the joints frames. (data.a)
        accel = robot_->acceleration(data_, robot_->model().getJointId("panda_joint7"));
    }

    void FrankaHuskyWrapper::acceleration_origin(pinocchio::Motion & accel){
        //API:Vector of joint accelerations expressed at the origin of the world. (data.oa)
        //It is not available!!!!!!!!!!!!!!! (always zero), so acceleratioon_global is used.
        accel = robot_->acceleration_origin(data_, robot_->model().getJointId("panda_joint7"));
    }

    void FrankaHuskyWrapper::acceleration_origin2(pinocchio::Motion & accel){
        //It is defined becuase data.oa is not available (output always zero)
        SE3 m_wMl;
        Motion a_frame;
        robot_->framePosition(data_, robot_->model().getFrameId("panda_joint7"), m_wMl);       //data.oMi
        robot_->frameAcceleration(data_, robot_->model().getFrameId("panda_joint7"), a_frame); //data.a
        accel = m_wMl.act(a_frame);
    }

    void FrankaHuskyWrapper::force(pinocchio::Force & force){
        //API:Vector of body forces expressed in the local frame of the joint.
        //API:For each body, the force represents the sum of all external forces acting on the body.
        force = robot_->force(data_, robot_->model().getJointId("panda_joint7"));
    }

    void FrankaHuskyWrapper::force_origin(pinocchio::Force & force){
        //API:Vector of body forces expressed in the world frame.
        //API:For each body, the force represents the sum of all external forces acting on the body.
        //It is not available!!!!!!!!!!!!!!! (always zero), so acceleratioon_global is used.
        force = robot_->force_origin(data_, robot_->model().getJointId("panda_joint7"));
    }

    void FrankaHuskyWrapper::force_origin2(pinocchio::Force & force){
        //It is defined becuase data.of is not available (output always zero)
        SE3 m_wMl;
        Force f_frame;
        robot_->framePosition(data_, robot_->model().getFrameId("panda_joint7"), m_wMl);
        robot_->frameForce(data_, robot_->model().getFrameId("panda_joint7"), f_frame);
        force = m_wMl.act(f_frame);
    }

    void FrankaHuskyWrapper::tau(VectorXd & tau_vec){
        //API:Vector of joint torques (dim model.nv).
        //It is not available (output always zero)
        if (ismobile_) tau_vec = robot_->jointTorques(data_).tail(na_-2);
        else           tau_vec = robot_->jointTorques(data_).tail(na_);
    }

    void FrankaHuskyWrapper::ddq(VectorXd & ddq_vec){
        //API:The joint accelerations computed from ABA.
        //It is not available (output always zero), will be available with ABA method
        //even though with ABA method, the value is not reasonable (behave like torque, not ddq)
        if (ismobile_) ddq_vec = robot_->jointAcceleration(data_).tail(na_-2);
        else           ddq_vec = robot_->jointAcceleration(data_).tail(na_);
    }

    void FrankaHuskyWrapper::mass(MatrixXd & mass_mat){
        if (ismobile_) mass_mat = robot_->mass(data_).bottomRightCorner(na_-2, na_-2);
        else           mass_mat = robot_->mass(data_).bottomRightCorner(na_, na_);
    }

    void FrankaHuskyWrapper::nle(VectorXd & nle_vec){
        if (ismobile_) nle_vec = robot_->nonLinearEffects(data_).tail(na_-2);
        else           nle_vec = robot_->nonLinearEffects(data_).tail(na_);
    }

    void FrankaHuskyWrapper::g(VectorXd & g_vec){
        //API:Vector of generalized gravity (dim model.nv).
        if (ismobile_) g_vec = data_.g.tail(na_-2);
        else           g_vec = data_.g.tail(na_);
    }

    void FrankaHuskyWrapper::g_joint7(VectorXd & g_vec){
        Vector3d g_global;
        // g_global << 0.0, 0.0, -9.81;
        g_global << 0.0, 0.0, 9.81;

        SE3 m_wMl;
        robot_->framePosition(data_, robot_->model().getFrameId("panda_joint7"), m_wMl);
        m_wMl.translation() << 0.0, 0.0, 0.0; //transform only with rotation
        g_vec = m_wMl.actInv(g_global);
    }

    void FrankaHuskyWrapper::g_local_offset(VectorXd & g_vec){
        Vector3d g_global;
        g_global << 0.0, 0.0, -9.81;
        //g_global << 0.0, 0.0, 9.81;

        SE3 m_wMl;
        m_wMl = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;
        m_wMl.translation() << 0.0, 0.0, 0.0; //transform only with rotation
        g_vec = m_wMl.actInv(g_global);
    }

    void FrankaHuskyWrapper::JWorld(MatrixXd & Jo){
        Data::Matrix6x Jo2;
        Jo2.resize(6, robot_->nv());
        robot_->jacobianWorld(data_, robot_->model().getJointId("panda_joint7"), Jo2);
        Jo = Jo2.bottomRightCorner(6, 7);
    }

    void FrankaHuskyWrapper::Fext_comp_arm_torque(VectorXd & tau){
        tau.setZero(7);
        if (!has_fext_)
            return;

        // Grasp-site wrench to arm joint load-compensation torque.
        MatrixXd J;          // 6 x 7 world arm jacobian at panda_joint7
        JWorld(J);
        // grasp site wrench를 joint7 기준 wrench로 변환한다: W_j7 = [F; T + d x F].
        const pinocchio::SE3 l7 = robot_->position(data_, robot_->model().getJointId("panda_joint7"));
        const Eigen::Vector3d d  = l7.rotation() * ee_offset_;
        const Eigen::Vector3d F  = Fext_.head(3);
        const Eigen::Vector3d Tq = Fext_.tail(3);
        Vector6d W;
        W.head(3) = F;
        W.tail(3) = Tq + d.cross(F);
        if (J.allFinite() && W.allFinite())
            tau = J.transpose() * W;        // 7-vector arm joint torque (load compensation)
    }

    void FrankaHuskyWrapper::JLocal(MatrixXd & Jo){
        Data::Matrix6x Jo2;
        Jo2.resize(6, robot_->nv());
        robot_->frameJacobianLocal(data_, robot_->model().getFrameId("panda_joint7"), Jo2);
        Jo = Jo2.bottomRightCorner(6, 7);
    }

    void FrankaHuskyWrapper::JLocal_offset(MatrixXd & Jo){
        Data::Matrix6x Jo2;
        Jo2.resize(6, robot_->nv());
        robot_->frameJacobianLocal(data_, robot_->model().getFrameId("panda_joint7"), Jo2);
        Jo = Jo2.bottomRightCorner(6, 7);
        Jo = Adj_mat_ * Jo;
    }

    void FrankaHuskyWrapper::dJLocal(MatrixXd & dJo){
        Data::Matrix6x dJo2;
        dJo2.resize(6, robot_->nv());
        robot_->frameJacobianTimeVariationLocal(data_, robot_->model().getFrameId("panda_joint7"), dJo2);
        dJo = dJo2.bottomRightCorner(6, 7);
    }

    void FrankaHuskyWrapper::dJLocal_offset(MatrixXd & dJo){
        Data::Matrix6x dJo2;
        dJo2.resize(6, robot_->nv());
        robot_->frameJacobianTimeVariationLocal(data_, robot_->model().getFrameId("panda_joint7"), dJo2);
        dJo = dJo2.bottomRightCorner(6, 7);
        dJo = Adj_mat_ * dJo;
    }

    void FrankaHuskyWrapper::ee_state(Vector3d & pos, Eigen::Quaterniond & quat){
        for (int i=0; i<3; i++)
            pos(i) = robot_->position(data_, robot_->model().getJointId("panda_joint7")).translation()(i);

        Quaternion<double> q(robot_->position(data_, robot_->model().getJointId("panda_joint7")).rotation());
        quat = q;
    }

    void FrankaHuskyWrapper::base_state(Vector3d & base){
        const SE3 H_mobile = robot_->getMobilePosition(data_, 5);
        base(0) = H_mobile.translation()(0);
        base(1) = H_mobile.translation()(1);
        base(2) = atan2(-H_mobile.rotation()(0, 1), H_mobile.rotation()(0, 0));
    }

    void FrankaHuskyWrapper::base_ref_state(Vector3d & base_ref){
        base_ref.setZero();
        if (!ismobile_) return;

        base_ref(0) = H_mobile_ref_.translation()(0);
        base_ref(1) = H_mobile_ref_.translation()(1);
        base_ref(2) = atan2(-H_mobile_ref_.rotation()(0, 1), H_mobile_ref_.rotation()(0, 0));
    }

    void FrankaHuskyWrapper::mobile_tracking_error(Vector3d & err){
        err.setZero();
        if (!ismobile_ || !mobileTask2_) return;

        const auto & pos_err = mobileTask2_->position_error();
        const int dim = std::min<int>(3, pos_err.size());
        for (int i = 0; i < dim; ++i)
            err(i) = pos_err(i);
    }

    void FrankaHuskyWrapper::mobile_velocity_error(Vector3d & err){
        err.setZero();
        if (!ismobile_ || !mobileTask2_) return;

        const auto & vel_err = mobileTask2_->velocity_error();
        const int dim = std::min<int>(3, vel_err.size());
        for (int i = 0; i < dim; ++i)
            err(i) = vel_err(i);
    }

    void FrankaHuskyWrapper::mobile_desired_acc(Vector3d & acc){
        acc.setZero();
        if (!ismobile_ || !mobileTask2_) return;

        const auto & des_acc = mobileTask2_->getDesiredAcceleration();
        const int dim = std::min<int>(3, des_acc.size());
        for (int i = 0; i < dim; ++i)
            acc(i) = des_acc(i);
    }

    void FrankaHuskyWrapper::mobile_command(Vector3d & cmd){
        cmd.setZero();
        if (!ismobile_ || state_.torque_.size() < 2) return;

        cmd(0) = state_.torque_(0);
        cmd(1) = state_.torque_(1);
        cmd(2) = state_.torque_(0) - state_.torque_(1);
    }

    void FrankaHuskyWrapper::rotz(double & angle, Eigen::Matrix3d & rot){
        rot.row(0) << cos(angle), -sin(angle),  0;
        rot.row(1) << sin(angle),  cos(angle),  0;
        rot.row(2) << 0,                    0,  1;
    }

    MatrixXd FrankaHuskyWrapper::skew_matrix(const VectorXd& vec){
        double v1 = vec(0);
        double v2 = vec(1);
        double v3 = vec(2);

        Eigen::Matrix3d CM;
        CM <<     0, -1*v3,    v2,
                 v3,     0, -1*v1,
              -1*v2,    v1,     0;

        return CM;
    }

    pinocchio::SE3 FrankaHuskyWrapper::vel_to_SE3(VectorXd vel, double dt){
        Quaterniond angvel_quat(1, vel(3) * dt * 0.5, vel(4) * dt * 0.5, vel(5) * dt * 0.5);
        Matrix3d Rotm = angvel_quat.normalized().toRotationMatrix();

        pinocchio::SE3 T;
        T.translation() = vel.head(3) * dt;
        T.rotation() = Rotm;

        // cout << T << endl;

        return T;
    }

    double FrankaHuskyWrapper::trajectory_length_in_time(){
        return  traj_length_in_time_;
    }

    double FrankaHuskyWrapper::noise_elimination(double x, double limit) {
        double y;
        if (abs(x) > limit) y = x;
        else y = 0.0;

        return y;
    }
}// namespace
