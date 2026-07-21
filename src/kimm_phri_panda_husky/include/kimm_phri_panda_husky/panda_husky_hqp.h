#pragma once

//Pinocchio Header
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

//ROS Header
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Transform.h"
#include "std_msgs/Int16.h"

//SYSTEM Header
#include <vector>

// for hqp controller
#include <kimm_hqp_controller/robot/robot_wrapper.hpp>
#include <kimm_hqp_controller/formulation/inverse_dynamics_formulation_acc.hpp>
#include <kimm_hqp_controller/tasks/task_se3_equality.hpp>
#include <kimm_hqp_controller/tasks/task_joint_posture.hpp>
#include <kimm_hqp_controller/tasks/task_joint_bound.hpp>
#include <kimm_hqp_controller/tasks/task_mobile_base.hpp>
#include <kimm_hqp_controller/trajectory/trajectory_euclidian.hpp>
#include <kimm_hqp_controller/trajectory/trajectory_se3.hpp>
#include <kimm_hqp_controller/solver/solver_HQP_factory.hxx>
// #include <kimm_hqp_controller/solver/util.hpp> //multiple definition problem
#include <kimm_hqp_controller/math/util.hpp>

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

typedef struct State {
    VectorXd q_;
    VectorXd v_;
    VectorXd dv_;
    VectorXd torque_;
    VectorXd tau_; //use to calculate ddq
} state;
namespace RobotController{
    class FrankaHuskyWrapper{
        public:
            FrankaHuskyWrapper(const std::string & robot_node, const bool & issimulation, const bool & ismobile, const bool & isrobotiq, ros::NodeHandle & node);
            ~FrankaHuskyWrapper(){};

            void initialize();
            void ctrl_update(const int& ); // msg for chaning controller
            void franka_update(const sensor_msgs::JointState&); // franka state update //for simulation
            void franka_update(const Vector7d&, const Vector7d&); // franka state update //for experiment
            void franka_update(const Vector7d& q, const Vector7d& qdot, const Vector7d& tau); // franka state update //for experiment
            void husky_update(const sensor_msgs::JointState&); // husky state update
            void husky_update(const Vector3d&, const Vector3d&, const Vector2d&, const Vector2d&); // husky state update
            void Fext_update(const Vector6d& Fext); // simulation/real gripper wrench input

            void compute(const double &); // computation by hqp controller
            void franka_output(VectorXd & qacc); // joint torque of franka
            void husky_output(VectorXd & qvel); // joint velocity of husky

            void position(pinocchio::SE3 & oMi);
            void elbow_position(pinocchio::SE3 & oMi);  // panda_joint4 진단 pose
            void position_offset(pinocchio::SE3 & oMi);
            void position_link0_offset(pinocchio::SE3 & oMi);
            void com(Eigen::Vector3d& com);
            void velocity(pinocchio::Motion& vel);
            void velocity_offset(pinocchio::Motion& vel);
            void velocity_origin(pinocchio::Motion& vel);
            void acceleration(pinocchio::Motion& accel);
            void acceleration_origin(pinocchio::Motion & accel);
            void acceleration_origin2(pinocchio::Motion & accel);
            void force(pinocchio::Force & force);
            void force_origin(pinocchio::Force & force);
            void force_origin2(pinocchio::Force & force);
            void tau(VectorXd & tau_vec);
            void ddq(VectorXd & ddq_vec);

            void mass(MatrixXd & mass_mat);
            void nle(VectorXd & nle_vec);
            void JWorld(MatrixXd & J);
            // EE 외력 Fext_를 arm joint torque로 변환해 하중 보상 실험에 사용한다.
            void Fext_comp_arm_torque(VectorXd & tau);
            void JLocal(MatrixXd & Jo);
            void JLocal_offset(MatrixXd & Jo);
            void dJLocal(MatrixXd & dJo);
            void dJLocal_offset(MatrixXd & dJo);
            void g(VectorXd & g_vec);
            void g_joint7(VectorXd & g_vec);
            void g_local_offset(VectorXd & g_vec);
            void ee_state(Vector3d & pos, Eigen::Quaterniond & quat);
            void base_state(Vector3d & base);
            void base_ref_state(Vector3d & base_ref);
            void mobile_tracking_error(Vector3d & err);
            void mobile_velocity_error(Vector3d & err);
            void mobile_desired_acc(Vector3d & acc);
            void mobile_command(Vector3d & cmd);

            void rotz(double & a, Eigen::Matrix3d & rot);

            MatrixXd skew_matrix(const VectorXd& vec);
            pinocchio::SE3 vel_to_SE3(VectorXd vel, double dt);
            double trajectory_length_in_time();
            double noise_elimination(double x, double limit);
            void eeoffset_update();

            int ctrltype(){
                return ctrl_mode_;
            }

            void state(State & state_robot){
                state_robot = state_;
            }
            bool reset_control_;

            // 협업 운반 orchestration node가 EE/base task reference를 주입한다.
            void set_ee_target(const pinocchio::SE3 & target);
            void set_base_target(double x, double y, double yaw);
            // CM seed를 posture reference로 사용해 redundant arm posture를 정한다.
            void set_arm_posture_target(const Eigen::VectorXd & q_arm);

        private:
            void mode235PeerReadyCallback(const std_msgs::Int16::ConstPtr & msg);
            void mode235GraspErrorCallback(const std_msgs::Float32MultiArray::ConstPtr & msg);
            void configureDefaultPostureTask();
            void mode235ConfigurePostureTask();
            double mode235CurrentPostureWeight() const;
            bool issimulation_, mode_change_;
            double time_;
            std::string robot_node_;
            State state_;
            int ctrl_mode_;
            Eigen::VectorXd q_ref_;
            pinocchio::SE3 H_ee_ref_, H_mobile_ref_, T_offset_;
            pinocchio::SE3 ee_target_ext_;
            Vector3d base_target_ext_{Vector3d::Zero()};
            bool has_ee_target_ext_{false};
            bool has_base_target_ext_{false};
            bool ee_target_dirty_{false};
            bool base_target_dirty_{false};
            Eigen::VectorXd arm_posture_target_ext_;
            bool has_arm_posture_target_ext_{false};
            bool arm_posture_target_dirty_{false};
            // mode 200: the last commanded EE reference sample is valid, so a
            // target re-plan can anchor on it instead of the measured pose
            // (re-anchoring on the measured pose absorbs tracking lag into the
            // reference and lets a persistent load ratchet the arm downward).
            bool ee_ref_sample_valid_{false};
            Vector3d ee_offset_;
            MatrixXd Adj_mat_;
            double joint7_to_finger_;
            Vector6d Fext_{Vector6d::Zero()};
            bool has_fext_{false};
            double traj_length_in_time_;
            double mobile_yaw_weight_{8.0}, mobile_xy_weight_{4.0};
            double mobile_yaw_kp_{400.0}, mobile_xy_kp_{100.0};
            double mode235_path_p1_x_{0.85}, mode235_path_p1_y_{0.30}, mode235_path_p1_theta_{40.0}, mode235_path_p1_duration_{10.0};
            double mode235_path_p2_x_{1.50}, mode235_path_p2_y_{0.30}, mode235_path_p2_theta_{25.0}, mode235_path_p2_duration_{12.0};
            double mode235_path_p3_x_{2.25}, mode235_path_p3_y_{0.15}, mode235_path_p3_theta_{-15.0}, mode235_path_p3_duration_{16.0};
            double mode235_path_p4_x_{3.00}, mode235_path_p4_y_{0.00}, mode235_path_p4_theta_{0.0}, mode235_path_p4_duration_{20.0};
            double mode235_path_p5_x_{6.00}, mode235_path_p5_y_{0.00}, mode235_path_p5_theta_{0.0}, mode235_path_p5_duration_{24.0};
            bool mode235_path_straight_path_{false};
            double mode235_path_straight_y_{0.0}, mode235_path_straight_theta_{0.0};
            double mode235_path_straight_x_scale_{1.0};
            double mode235_path_straight_p1_duration_{-1.0}, mode235_path_straight_p2_duration_{-1.0};
            double mode235_path_straight_p3_duration_{-1.0}, mode235_path_straight_p4_duration_{-1.0};
            bool mode235_path_relative_x_{false}, mode235_path_relative_y_{false};
            double mode235_path_x_anchor_{0.0}, mode235_path_y_anchor_{0.0};
            double mode235_pre_straight_x_{3.0}, mode235_pre_straight_duration_{24.0};
            double mode235_leader_posture_weight_{1.00};
            double mode235_leader_ee_pos_kp_{40.0}, mode235_leader_ee_ori_kp_{80.0};
            double mode235_follower_posture_weight_{0.10};
            double mode235_follower_ee_pos_kp_{120.0}, mode235_follower_ee_ori_kp_{180.0};
            double mode235_follower_ee_kd_ratio_{1.0};
            double mode235_phase_xy_tolerance_{0.20}, mode235_phase3_xy_tolerance_{0.20}, mode235_phase_yaw_tolerance_deg_{20.0};
            Eigen::Vector3d mode235_mobile_to_ee_offset_{Eigen::Vector3d::Zero()};
            Eigen::Matrix3d mode235_ee_rotation_ref_{Eigen::Matrix3d::Identity()};
            double mode235_ee_z_ref_{0.0};
            double mode235_latest_leader_ee_z_{0.0}, mode235_latest_follower_ee_z_{0.0};
            bool mode235_has_lift_height_metrics_{false};
            int mode235_own_ready_phase_{-1}, mode235_peer_ready_phase_{-1};
            int mode235_path_phase_{0};
            int mode235_path_done_phase_{-1};
            double mode235_path_phase_start_time_{0.0};
            VectorXd posture_gain_default_;
            bool ismobile_, isrobotiq_;
            //hqp
            std::shared_ptr<kimmhqp::robot::RobotWrapper> robot_;
            pinocchio::Model model_;
            pinocchio::Data data_;

            std::shared_ptr<kimmhqp::InverseDynamicsFormulationAccForce> tsid_;

            std::shared_ptr<kimmhqp::tasks::TaskJointPosture> postureTask_;
            std::shared_ptr<kimmhqp::tasks::TaskSE3Equality> eeTask_;
            // elbow z task: EE task를 유지한 채 null-space posture를 조정한다.
            std::shared_ptr<kimmhqp::tasks::TaskSE3Equality> elbowTask_;
            double elbow_z_target_{-1.0};
            bool   elbow_task_added_{false};
        public:
            void set_elbow_z_target(double z){ elbow_z_target_ = z; }
        private:
            std::shared_ptr<kimmhqp::tasks::TaskJointBounds> torqueBoundsTask_;
            std::shared_ptr<kimmhqp::tasks::TaskMobileEquality> mobileTask_, mobileTask2_;

            std::shared_ptr<kimmhqp::trajectory::TrajectoryEuclidianCubic> trajPosture_Cubic_;
            std::shared_ptr<kimmhqp::trajectory::TrajectoryEuclidianConstant> trajPosture_Constant_;
            std::shared_ptr<kimmhqp::trajectory::TrajectoryEuclidianTimeopt> trajPosture_Timeopt_;
            std::shared_ptr<kimmhqp::trajectory::TrajectorySE3Cubic> trajEE_Cubic_, trajMobile_Cubic_;
            std::shared_ptr<kimmhqp::trajectory::TrajectorySE3Constant> trajEE_Constant_, trajMobile_Constant_;
            std::shared_ptr<kimmhqp::trajectory::TrajectorySE3Timeopt> trajEE_Timeopt_, trajMobile_Timeopt_;

            kimmhqp::trajectory::TrajectorySample sampleMobile_, sampleEE_, samplePosture_;

            kimmhqp::solver::SolverHQPBase * solver_;

            int na_, nq_, nv_;
            int cnt_;

            //ros
            ros::Publisher mode235_phase_ready_pub_;
            ros::Subscriber mode235_peer_ready_sub_, mode235_grasp_error_sub_;
            ros::NodeHandle n_node_;
    };
} // namespace
