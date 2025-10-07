// #include "state_estimator/plugin.hpp"
// #include "state_estimator/Models/attitude_bias_NLO.hpp"
// #include "state_estimator/Models/attitude_bias_XKF.hpp"
// // #include "iit/commons/geometry/rotations.h"

// #include "state_estimator_msgs/msg/attitude.hpp"
// #include "state_estimator/lib.hpp"

// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <rosgraph_msgs/msg/clock.hpp>
// #include <sensor_msgs/msg/imu.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <std_msgs/msg/int32.hpp>
// #include <std_msgs/msg/string.hpp>

// #include <iostream>
// #include <rclcpp/rclcpp.hpp>
// #include <chrono>
// #include <functional>
// #include <cmath>

// // Eigen type aliases
// using Matrix6d = Eigen::Matrix<double, 6, 6>;
// using Vector6d = Eigen::Matrix<double, 6, 1>;
// using Vector7d = Eigen::Matrix<double, 7, 1>;

// namespace state_estimator_plugins 
// {
//     class AttitudeEstimationPlugin : public PluginBase 
//     {
//     public:

//         AttitudeEstimationPlugin():
//         attitude_(nullptr)
//         { } 	

//         ~AttitudeEstimationPlugin() 
//         {
//             if (attitude_!=nullptr) delete(attitude_);
//         }
        
//         std::string getName() override { return std::string("AttitudeEstimation"); }
// 		std::string getDescription() override { return std::string("Attitude Estimation Plugin"); }

//         void initialize_() override 
//         {
//             t0 = 0.0;
//             xhat_estimated << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//             xhat_estimated.head(4) = xhat_estimated.head(4) / xhat_estimated.head(4).norm();

//             // Load config parameters
//             std::string imu_topic,pub_topic;
//             std::vector<double> base_R_imu_vec, north_vec, gravity_vec;
//             std::vector<double> P_vec, Q_vec, R_vec;
//             double ki_param, kp_param;

//             // ROS 2 parameter declaration and retrieval
//             RCLCPP_INFO(node_->get_logger(), "Declaring attitude plugin parameters...");
            
//             // imu_topic = node_->declare_parameter("attitude_estimation_plugin.imu_topic", std::string("/sensors/imu"));
//             imu_topic = "/sensors/imu";
//             RCLCPP_INFO_STREAM(node_->get_logger(), "imu_topic: " << imu_topic);
            
//             // pub_topic = node_->declare_parameter("attitude_estimation_plugin.pub_topic", std::string("attitude"));
//             pub_topic = "attitude";
//             RCLCPP_INFO_STREAM(node_->get_logger(), "pub_topic: " << pub_topic);
            
//             // ki_param = node_->declare_parameter("attitude_estimation_plugin.ki", 0.02);
//             ki_param = 0.02;
//             RCLCPP_INFO_STREAM(node_->get_logger(), "ki: " << ki_param);
            
//             // kp_param = node_->declare_parameter("attitude_estimation_plugin.kp", 10.0);
//             kp_param=10.0;
//             RCLCPP_INFO_STREAM(node_->get_logger(), "kp: " << kp_param);
            
//             // base_R_imu_vec = node_->declare_parameter("attitude_estimation_plugin.base_R_imu", std::vector<double>());
//             base_R_imu_vec = {-1.0, 0.0, 0.0, 
//                               0.0, 1.0, 0.0, 
//                               0.0, 0.0, -1.0};
//             RCLCPP_INFO_STREAM(node_->get_logger(), "base_R_imu size: " << base_R_imu_vec.size());
            
//             // north_vec = node_->declare_parameter("attitude_estimation_plugin.north_vector", std::vector<double>());
//             north_vec = {0.577, 0.577, 0.577};
//             RCLCPP_INFO_STREAM(node_->get_logger(), "north_vector size: " << north_vec.size());
            
//             // gravity_vec = node_->declare_parameter("attitude_estimation_plugin.gravity_vector", std::vector<double>());
//             gravity_vec = {0.0, 0.0, 9.81};
//             RCLCPP_INFO_STREAM(node_->get_logger(), "gravity_vector size: " << gravity_vec.size());
            
//             // P_vec = node_->declare_parameter("attitude_estimation_plugin.P", std::vector<double>());
//             P_vec = {
//                 1.0e-6, 0.0, 0.0, 0.0, 0.0, 0.0,
//                 0.0, 1.0e-6, 0.0, 0.0, 0.0, 0.0,
//                 0.0, 0.0, 1.0e-6, 0.0, 0.0, 0.0,
//                 0.0, 0.0, 0.0, 1.0e-6, 0.0, 0.0,
//                 0.0, 0.0, 0.0, 0.0, 1.0e-6, 0.0,
//                 0.0, 0.0, 0.0, 0.0, 0.0, 1.0e-6
//             };
//             RCLCPP_INFO_STREAM(node_->get_logger(), "P matrix size: " << P_vec.size());
            
//             // Q_vec = node_->declare_parameter("attitude_estimation_plugin.Q", std::vector<double>());
//             Q_vec = {
//                 1.0e-6, 0.0, 0.0, 0.0, 0.0, 0.0,
//                 0.0, 1.0e-6, 0.0, 0.0, 0.0, 0.0,
//                 0.0, 0.0, 1.0e-6, 0.0, 0.0, 0.0,
//                 0.0, 0.0, 0.0, 1.0e-8, 0.0, 0.0,
//                 0.0, 0.0, 0.0, 0.0, 1.0e-8, 0.0,
//                 0.0, 0.0, 0.0, 0.0, 0.0, 1.0e-8
//             };
//             RCLCPP_INFO_STREAM(node_->get_logger(), "Q matrix size: " << Q_vec.size());
            
//             // R_vec = node_->declare_parameter("attitude_estimation_plugin.R", std::vector<double>());
//             R_vec = {
//                 4.0e-2, 0.0, 0.0, 0.0, 0.0, 0.0,
//                 0.0, 4.0e-2, 0.0, 0.0, 0.0, 0.0,
//                 0.0, 0.0, 4.0e-2, 0.0, 0.0, 0.0,
//                 0.0, 0.0, 0.0, 16.0, 0.0, 0.0,
//                 0.0, 0.0, 0.0, 0.0, 16.0, 0.0,
//                 0.0, 0.0, 0.0, 0.0, 0.0, 16.0
//             };
//             RCLCPP_INFO_STREAM(node_->get_logger(), "R matrix size: " << R_vec.size());

//             // Set gains
//             ki = ki_param;
//             kp = kp_param;

//             // Convert base_R_imu
//             if (base_R_imu_vec.size() == 9) {
//                 b_R_imu = Eigen::Map<Eigen::Matrix3d>(base_R_imu_vec.data());
//             } else {
//                 RCLCPP_WARN(node_->get_logger(), "Invalid or missing base_R_imu config — using identity.");
//                 b_R_imu.setIdentity();
//             }

//             // Convert north_vector
//             if (north_vec.size() == 3) {
//                 m_n = Eigen::Map<Eigen::Vector3d>(north_vec.data());
//             } else {
//                 RCLCPP_WARN(node_->get_logger(), "Invalid or missing north_vector config — using default.");
//                 m_n << 1.0 / sqrt(3), 1.0 / sqrt(3), 1.0 / sqrt(3);
//             }

//             // Convert gravity_vector
//             if (gravity_vec.size() == 3) {
//                 f_n = Eigen::Map<Eigen::Vector3d>(gravity_vec.data());
//             } else {
//                 RCLCPP_WARN(node_->get_logger(), "Invalid or missing gravity_vector config — using default.");
//                 f_n << 0.0, 0.0, 9.81;
//             }
            
//             // Convert and validate matrices
//             Matrix6d P0, Q, R;
            
//             if (P_vec.size() == 36) {
//                 P0 = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(P_vec.data());
//             } else {
//                 RCLCPP_WARN(node_->get_logger(), "Invalid or missing P matrix config — using default identity.");
//                 P0.setIdentity();
//                 P0 *= 1e-6;
//             }
            
//             if (Q_vec.size() == 36) {
//                 Q = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(Q_vec.data());
//             } else {
//                 RCLCPP_WARN(node_->get_logger(), "Invalid or missing Q matrix config — using default identity.");
//                 Q.setIdentity();
//                 Q *= 1e-6;
//             }
            
//             if (R_vec.size() == 36) {
//                 R = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(R_vec.data());
//             } else {
//                 RCLCPP_WARN(node_->get_logger(), "Invalid or missing R matrix config — using default identity.");
//                 R.setIdentity();
//                 R *= 1e-2;
//             }

//             attitude_ = new state_estimator::AttitudeBiasXKF(t0, xhat_estimated, P0, Q, R, f_n, m_n, ki, kp);

//             // Set up ROS 2 interfaces
//             imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
//                 imu_topic, 250,
//                 std::bind(&AttitudeEstimationPlugin::callback_imu, this, std::placeholders::_1));
                
//             pub_ = node_->create_publisher<state_estimator_msgs::msg::Attitude>(pub_topic, 1);

//             RCLCPP_INFO_STREAM(node_->get_logger(), "AttitudeEstimationPlugin initialized with topic '" << imu_topic << "'");
                
//         }
// 		void shutdown_() override { }
// 		void pause_() override { }
// 		void resume_() override { }
// 		void reset_() override { }

//         void callback_imu 
//         (
//             const sensor_msgs::msg::Imu::SharedPtr imu
//         )
//         {
//             static int callback_count = 0;
//             callback_count++;
            
//             if (callback_count % 100 == 0) {
//                 RCLCPP_INFO(node_->get_logger(), "IMU callback called %d times", callback_count);
//             }
            
//             Eigen::Vector3d omega(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
//             Eigen::Vector3d acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);	
//             computeAttitude(omega,acc);		
//         }
        
//         void computeAttitude(Eigen::Vector3d &omega,Eigen::Vector3d &acc)
// 		{
// 			/* to take the time from a rosbag:
// 			$ roscore
// 			$ rosparam set use_sim_time true
// 			$ rosbag play name_of_the_bag.bag --clock
// 			*/
	
//             if (begin)
//             {
//                 time_begin_ = node_->get_clock()->now();
//                 begin = false;
//             }		

//             auto current_time = node_->get_clock()->now();
//             time_ = (current_time - time_begin_).seconds();
            
//             quat_est.w() = xhat_estimated(0);
//       		quat_est.vec() << xhat_estimated(1),xhat_estimated(2),xhat_estimated(3);

//             f_b = b_R_imu*acc;
//             m_b = quat_est.toRotationMatrix()*m_n;

//             z << f_b, m_b;
//             attitude_->update(time_,b_R_imu*omega, z);

//             xhat_estimated = attitude_->getX();
			
//             xdot = attitude_->calc_f(time_,xhat_estimated,b_R_imu*omega);
//             quat_dot.w() = xdot(0);
//             quat_dot.vec() << xdot(1),xdot(2),xdot(3);
            
//             // Calculate omega_filt using quaternion derivative
//             // omega_filt = 2 * quat_est.conjugate() * quat_dot (in vector form)
//             Eigen::Quaterniond quat_conj = quat_est.conjugate();
//             Eigen::Quaterniond temp = quat_conj * quat_dot;
//             omega_filt << temp.x(), temp.y(), temp.z();
//             omega_filt *= 2.0;
            
//             // Publishing
//             msg_.header.stamp = current_time;
//             msg_.header.frame_id = "base_link";  // Add frame_id
//             msg_.quaternion[0] = quat_est.w();
// 			msg_.quaternion[1] = quat_est.x();
// 			msg_.quaternion[2] = quat_est.y();
// 			msg_.quaternion[3] = quat_est.z();

//             // Convert quaternion to Euler angles (Roll, Pitch, Yaw)
//             // Using standard conversion: q = [w, x, y, z]
//             double w = quat_est.w();
//             double x = quat_est.x();
//             double y = quat_est.y();
//             double z = quat_est.z();
            
//             // Roll (x-axis rotation)
//             double sinr_cosp = 2 * (w * x + y * z);
//             double cosr_cosp = 1 - 2 * (x * x + y * y);
//             euler_radians(0) = std::atan2(sinr_cosp, cosr_cosp);
            
//             // Pitch (y-axis rotation)
//             double sinp = 2 * (w * y - z * x);
//             if (std::abs(sinp) >= 1)
//                 euler_radians(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
//             else
//                 euler_radians(1) = std::asin(sinp);
            
//             // Yaw (z-axis rotation)
//             double siny_cosp = 2 * (w * z + x * y);
//             double cosy_cosp = 1 - 2 * (y * y + z * z);
//             euler_radians(2) = std::atan2(siny_cosp, cosy_cosp);
            
//             euler_degrees = euler_radians * (180.0 / M_PI);
            
//             msg_.roll_deg  = euler_degrees(0);
//             msg_.pitch_deg = euler_degrees(1);
//             msg_.yaw_deg   = euler_degrees(2);
            
//             msg_.angular_velocity[0] = omega_filt(0);
//             msg_.angular_velocity[1] = omega_filt(1);
//             msg_.angular_velocity[2] = omega_filt(2);

//             pub_->publish(msg_);
            
//             static int publish_count = 0;
//             publish_count++;
            
//             if (publish_count % 100 == 0) {
//                 RCLCPP_INFO(node_->get_logger(), "Published %d attitude messages. Roll: %.2f, Pitch: %.2f, Yaw: %.2f", 
//                     publish_count, msg_.roll_deg, msg_.pitch_deg, msg_.yaw_deg);
//             }

// 		} // end computeAttitude


//     private:

//         state_estimator::AttitudeBiasXKF *attitude_;
//         Vector7d xhat_estimated;
//         Matrix6d P0;
//         Matrix6d Q;
//         Matrix6d R;
//         Eigen::Matrix<double,3,1> f_n; 
//         Eigen::Matrix<double,3,1> m_n; 
//         Eigen::Matrix3d b_R_imu;
//         double ki; 
//         double kp; 
//         double t0; 
    
//         rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
//         rclcpp::Publisher<state_estimator_msgs::msg::Attitude>::SharedPtr pub_;

//         Eigen::Quaterniond quat_est;
//         Eigen::Vector3d f_b;
//         Eigen::Vector3d m_b; 
//         Vector6d z;
//         Vector7d xdot;
//         Eigen::Quaterniond quat_dot;
//         Eigen::Vector3d omega_filt;
//         Eigen::Vector3d euler_radians;
//         Eigen::Vector3d euler_degrees;

//         state_estimator_msgs::msg::Attitude msg_;
    
//         double time_{};
//         bool begin{true};
//         rclcpp::Time time_begin_;
        
//     }; // end class AttitudeEstimationPlugin

// } // end namespace state_estimator_plugins

// #include <pluginlib/class_list_macros.hpp>
// PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::AttitudeEstimationPlugin, state_estimator_plugins::PluginBase)


// attitude_estimation_plugin.cpp
#include "state_estimator/plugin.hpp"
#include "state_estimator/Models/attitude_bias_NLO.hpp"
#include "state_estimator/Models/attitude_bias_XKF.hpp"

#include "state_estimator_msgs/msg/attitude.hpp"
#include "state_estimator/lib.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <Eigen/Dense>

#include <chrono>
#include <functional>
#include <cmath>
#include <iostream>

// Eigen type aliases
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

namespace state_estimator_plugins 
{
    class AttitudeEstimationPlugin : public PluginBase 
    {
    public:

        AttitudeEstimationPlugin():
            attitude_(nullptr),
            ki(0.0),
            kp(0.0),
            t0(0.0),
            begin(true)
        { }     

        ~AttitudeEstimationPlugin() 
        {
            if (attitude_ != nullptr) {
                delete attitude_;
                attitude_ = nullptr;
            }
        }
        
        std::string getName() override { return std::string("AttitudeEstimation"); }
        std::string getDescription() override { return std::string("Attitude Estimation Plugin"); }

        void initialize_() override 
        {
            // initialize state guess
            xhat_estimated.setZero();
            xhat_estimated(0) = 1.0; // q_w = 1
            xhat_estimated.head(4) = xhat_estimated.head(4) / xhat_estimated.head(4).norm();

            // Defaults
            std::string imu_topic = "/sensors/imu";
            std::string pub_topic = "attitude";
            double ki_param = 0.02;
            double kp_param = 10.0;

            std::vector<double> base_R_imu_vec = {
                -1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0,-1.0
            };
            std::vector<double> north_vec = {0.577, 0.577, 0.577};
            std::vector<double> gravity_vec = {0.0, 0.0, 9.81};

            // Reasonable defaults for P/Q/R (row-major)
            std::vector<double> P_vec(36, 0.0), Q_vec(36, 0.0), R_vec(36, 0.0);
            for (int i=0;i<6;i++) {
                P_vec[i*6 + i] = 1e-6;
                Q_vec[i*6 + i] = (i<3) ? 1e-6 : 1e-8;
                R_vec[i*6 + i] = (i<3) ? 4e-2 : 16.0;
            }

            // You may replace the above defaults by reading parameters from node_ like:
            // imu_topic = node_->declare_parameter("attitude_estimation_plugin.imu_topic", imu_topic);
            // (Left commented because plugin base may manage param declaration differently.)

            // set gains
            ki = ki_param;
            kp = kp_param;

            // base_R_imu
            if (base_R_imu_vec.size() == 9) {
                b_R_imu = Eigen::Map<Eigen::Matrix3d>(base_R_imu_vec.data());
            } else {
                RCLCPP_WARN(node_->get_logger(), "Invalid or missing base_R_imu — using identity.");
                b_R_imu.setIdentity();
            }

            // north vector
            if (north_vec.size() == 3) {
                m_n = Eigen::Map<Eigen::Vector3d>(north_vec.data());
            } else {
                RCLCPP_WARN(node_->get_logger(), "Invalid or missing north_vector — using default.");
                m_n << 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0);
            }

            // gravity vector
            if (gravity_vec.size() == 3) {
                f_n = Eigen::Map<Eigen::Vector3d>(gravity_vec.data());
            } else {
                RCLCPP_WARN(node_->get_logger(), "Invalid or missing gravity_vector — using default.");
                f_n << 0.0, 0.0, 9.81;
            }

            // Convert P/Q/R vectors into matrices (class members)
            if (P_vec.size() == 36) {
                P0 = Eigen::Map<const Matrix6d>(P_vec.data());
            } else {
                P0.setIdentity();
                P0 *= 1e-6;
            }

            if (Q_vec.size() == 36) {
                Q = Eigen::Map<const Matrix6d>(Q_vec.data());
            } else {
                Q.setIdentity();
                Q *= 1e-6;
            }

            if (R_vec.size() == 36) {
                R = Eigen::Map<const Matrix6d>(R_vec.data());
            } else {
                R.setIdentity();
                R *= 1e-2;
            }

            // instantiate the estimator
            attitude_ = new state_estimator::AttitudeBiasXKF(t0, xhat_estimated, P0, Q, R, f_n, m_n, ki, kp);

            // subscriptions / publishers using node_
            imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
                imu_topic, rclcpp::QoS(250),
                std::bind(&AttitudeEstimationPlugin::callback_imu, this, std::placeholders::_1)
            );

            pub_ = node_->create_publisher<state_estimator_msgs::msg::Attitude>(pub_topic, rclcpp::QoS(1));

            RCLCPP_INFO_STREAM(node_->get_logger(), "AttitudeEstimationPlugin initialized. IMU topic: " << imu_topic);
        }

        void shutdown_() override { }
        void pause_() override { }
        void resume_() override { }
        void reset_() override { }

        void callback_imu(const sensor_msgs::msg::Imu::SharedPtr imu)
        {
            // minimal periodic logging
            static int cb_count = 0;
            if ((++cb_count % 500) == 0) {
                RCLCPP_DEBUG(node_->get_logger(), "IMU callback received %d times", cb_count);
            }

            Eigen::Vector3d omega(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
            Eigen::Vector3d acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
            computeAttitude(omega, acc);
        }

        void computeAttitude(Eigen::Vector3d &omega, Eigen::Vector3d &acc)
        {
            if (begin)
            {
                time_begin_ = node_->get_clock()->now();
                begin = false;
            }

            rclcpp::Time current_time = node_->get_clock()->now();
            time_ = (current_time - time_begin_).seconds();

            // quaternion state estimate
            quat_est.w() = xhat_estimated(0);
            quat_est.x() = xhat_estimated(1);
            quat_est.y() = xhat_estimated(2);
            quat_est.z() = xhat_estimated(3);

            // measured vectors in body (transform IMU accel by base_R_imu)
            f_b = b_R_imu * acc;
            // magnetic measurement estimated from quaternion
            m_b = quat_est.toRotationMatrix() * m_n;

            // measurement vector z = [f_b; m_b]
            z.head<3>() = f_b;
            z.tail<3>() = m_b;

            // call the estimator (assuming signature update(time, omega_in_body, z))
            attitude_->update(time_, b_R_imu * omega, z);

            // fetch new estimate
            xhat_estimated = attitude_->getX();

            // compute derivative and (filtered) angular velocity
            xdot = attitude_->calc_f(time_, xhat_estimated, b_R_imu * omega);

            quat_dot.w() = xdot(0);
            quat_dot.x() = xdot(1);
            quat_dot.y() = xdot(2);
            quat_dot.z() = xdot(3);

            // Compute omega_filt from quaternion derivative:
            // For unit quaternion q, q_dot = 0.5 * Omega(q) * omega -> a common inversion yields:
            // omega ≈ 2 * (q_conj * q_dot).vec()
            Eigen::Quaterniond q_conj = quat_est.conjugate();
            Eigen::Quaterniond tmp = q_conj * quat_dot;
            omega_filt << tmp.x(), tmp.y(), tmp.z();
            omega_filt *= 2.0;

            // Build and publish message
            // msg_.header.stamp = node_->get_clock()->now().to_msg();
            // get current time
            rclcpp::Time now = node_->get_clock()->now();
            // convert to integer nanoseconds since epoch
            int64_t ns = now.nanoseconds();
            // fill builtin_interfaces::msg::Time (sec, nanosec)
            msg_.header.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
            msg_.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
            msg_.header.frame_id = "base_link";

            // fill quaternion (msg layout assumed: std::array<double,4> quaternion)
            msg_.quaternion[0] = quat_est.w();
            msg_.quaternion[1] = quat_est.x();
            msg_.quaternion[2] = quat_est.y();
            msg_.quaternion[3] = quat_est.z();

            // convert quaternion to Euler (radians)
            const double PI = std::acos(-1.0);
            double w = quat_est.w(), x = quat_est.x(), y = quat_est.y(), z_ = quat_est.z();

            double sinr_cosp = 2.0 * (w * x + y * z_);
            double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
            euler_radians(0) = std::atan2(sinr_cosp, cosr_cosp);

            double sinp = 2.0 * (w * y - z_ * x);
            if (std::fabs(sinp) >= 1.0)
                euler_radians(1) = std::copysign(PI / 2.0, sinp); // use 90 degrees if out of range
            else
                euler_radians(1) = std::asin(sinp);

            double siny_cosp = 2.0 * (w * z_ + x * y);
            double cosy_cosp = 1.0 - 2.0 * (y * y + z_ * z_);
            euler_radians(2) = std::atan2(siny_cosp, cosy_cosp);

            euler_degrees = euler_radians * (180.0 / PI);

            msg_.roll_deg  = euler_degrees(0);
            msg_.pitch_deg = euler_degrees(1);
            msg_.yaw_deg   = euler_degrees(2);

            msg_.angular_velocity[0] = omega_filt(0);
            msg_.angular_velocity[1] = omega_filt(1);
            msg_.angular_velocity[2] = omega_filt(2);

            pub_->publish(msg_);

            static int pub_count = 0;
            if ((++pub_count % 100) == 0) {
                RCLCPP_INFO(node_->get_logger(), "Published attitude #%d  (roll=%.2f pitch=%.2f yaw=%.2f)",
                    pub_count, msg_.roll_deg, msg_.pitch_deg, msg_.yaw_deg);
            }
        }

    private:

        state_estimator::AttitudeBiasXKF *attitude_;

        Vector7d xhat_estimated;
        Matrix6d P0;
        Matrix6d Q;
        Matrix6d R;
        Eigen::Vector3d f_n; 
        Eigen::Vector3d m_n; 
        Eigen::Matrix3d b_R_imu;

        double ki; 
        double kp; 
        double t0; 

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Publisher<state_estimator_msgs::msg::Attitude>::SharedPtr pub_;

        Eigen::Quaterniond quat_est;
        Eigen::Vector3d f_b;
        Eigen::Vector3d m_b; 
        Vector6d z;
        Vector7d xdot;
        Eigen::Quaterniond quat_dot;
        Eigen::Vector3d omega_filt;
        Eigen::Vector3d euler_radians;
        Eigen::Vector3d euler_degrees;

        state_estimator_msgs::msg::Attitude msg_;

        double time_{};
        bool begin;
        rclcpp::Time time_begin_;
    };

} // namespace state_estimator_plugins

PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::AttitudeEstimationPlugin, state_estimator_plugins::PluginBase)
