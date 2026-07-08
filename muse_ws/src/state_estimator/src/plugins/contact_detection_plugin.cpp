#include "state_estimator/Models/grf_contact_estimator.hpp"
#include "state_estimator/Models/joint_state_snapshot.hpp"
#include "state_estimator/plugin.hpp"

#include <anymal_msgs/AnymalState.h>
#include <state_estimator_msgs/ContactDetection.h>

#include <ros/package.h>
#include <ros/ros.h>

#include <array>
#include <exception>
#include <string>
#include <vector>

namespace state_estimator_plugins
{

class ContactDetectionPlugin : public PluginBase
{
public:
    ContactDetectionPlugin() = default;
    ~ContactDetectionPlugin() override = default;

    std::string getName() override
    {
        return std::string("ContactDetection");
    }

    std::string getDescription() override
    {
        return std::string("AnymalState GRF contact detection plugin");
    }

    void initialize_() override
    {

        std::string pub_topic;
        nh_.param(
            "contact_detection_plugin/pub_topic",
            pub_topic,
            std::string("contact_detection")
        );

        nh_.param(
            "contact_detection_plugin/grf_threshold",
            grf_threshold_,
            40.0
        );

        pub_ = nh_.advertise<state_estimator_msgs::ContactDetection>(
            pub_topic,
            250
        );

        initializeAnymalStateInput();
    }

    void shutdown_() override
    {
        anymal_state_sub_.shutdown();
        pub_.shutdown();
    }

    void pause_() override {}
    void resume_() override {}
    void reset_() override {}

private:
    void initializeAnymalStateInput()
    {
        std::string anymal_state_topic;
        std::string urdf_path_param;

        nh_.param(
            "contact_detection_plugin/anymal_state_topic",
            anymal_state_topic,
            std::string("/anymal/state_estimator/anymal_state")
        );

        nh_.param(
            "contact_detection_plugin/urdf_path",
            urdf_path_param,
            std::string("$(find state_estimator)/urdfs/anymal.urdf")
        );

        ROS_INFO_STREAM(
            "ContactDetectionPlugin subscribing to AnymalState topic: "
            << anymal_state_topic
        );

        anymal_state_sub_ = nh_.subscribe(
            anymal_state_topic,
            250,
            &ContactDetectionPlugin::anymalStateCallback,
            this
        );

        const std::string resolved_urdf_path = resolveRosFind(urdf_path_param);

        ROS_INFO_STREAM(
            "ContactDetectionPlugin using URDF path: "
            << resolved_urdf_path
        );

        std::vector<std::string> foot_frame_names_vec;
        nh_.param(
            "contact_detection_plugin/foot_frame_names",
            foot_frame_names_vec,
            std::vector<std::string>{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"}
        );

        const std::array<std::string, 4> foot_frame_names = toStringArray4(
            foot_frame_names_vec,
            {{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"}},
            "foot_frame_names"
        );

        std::vector<std::string> lf_joint_names;
        std::vector<std::string> rf_joint_names;
        std::vector<std::string> lh_joint_names;
        std::vector<std::string> rh_joint_names;

        nh_.param(
            "contact_detection_plugin/lf_joint_names",
            lf_joint_names,
            std::vector<std::string>{"LF_HAA", "LF_HFE", "LF_KFE"}
        );

        nh_.param(
            "contact_detection_plugin/rf_joint_names",
            rf_joint_names,
            std::vector<std::string>{"RF_HAA", "RF_HFE", "RF_KFE"}
        );

        nh_.param(
            "contact_detection_plugin/lh_joint_names",
            lh_joint_names,
            std::vector<std::string>{"LH_HAA", "LH_HFE", "LH_KFE"}
        );

        nh_.param(
            "contact_detection_plugin/rh_joint_names",
            rh_joint_names,
            std::vector<std::string>{"RH_HAA", "RH_HFE", "RH_KFE"}
        );

        const std::array<std::vector<std::string>, 4> leg_joint_names{{
            lf_joint_names,
            rf_joint_names,
            lh_joint_names,
            rh_joint_names
        }};

        std::vector<double> contact_normal_vec;
        nh_.param(
            "contact_detection_plugin/contact_normal",
            contact_normal_vec,
            std::vector<double>{0.0, 0.0, 1.0}
        );

        const Eigen::Vector3d contact_normal = toVector3(
            contact_normal_vec,
            Eigen::Vector3d::UnitZ(),
            "contact_normal"
        );

        double threshold_low;
        double threshold_high;
        double force_sign;

        bool use_hysteresis;
        bool use_force_norm;
        bool use_absolute_normal_force;
        bool use_inverse_dynamics_compensation;

        nh_.param(
            "contact_detection_plugin/grf_threshold_low",
            threshold_low,
            grf_threshold_
        );

        nh_.param(
            "contact_detection_plugin/grf_threshold_high",
            threshold_high,
            grf_threshold_
        );

        nh_.param(
            "contact_detection_plugin/use_hysteresis",
            use_hysteresis,
            false
        );

        nh_.param(
            "contact_detection_plugin/use_force_norm",
            use_force_norm,
            false
        );

        nh_.param(
            "contact_detection_plugin/use_absolute_normal_force",
            use_absolute_normal_force,
            true
        );

        nh_.param(
            "contact_detection_plugin/use_inverse_dynamics_compensation",
            use_inverse_dynamics_compensation,
            true
        );

        nh_.param(
            "contact_detection_plugin/force_sign",
            force_sign,
            -1.0
        );

        try
        {
            estimator_configured_ = grf_contact_estimator_.configure(
                resolved_urdf_path,
                foot_frame_names,
                leg_joint_names,
                grf_threshold_,
                threshold_low,
                threshold_high,
                use_hysteresis,
                use_force_norm,
                use_absolute_normal_force,
                force_sign,
                use_inverse_dynamics_compensation,
                contact_normal
            );

            if (!estimator_configured_)
            {
                ROS_ERROR_STREAM(
                    "Failed to configure GRF contact estimator: "
                    << grf_contact_estimator_.lastError()
                );
                return;
            }

        }
        catch (const std::exception& e)
        {
            estimator_configured_ = false;

            ROS_ERROR_STREAM(
                "Failed to configure GRF contact estimator: "
                << e.what()
            );

            return;
        }

        ROS_INFO_STREAM(
            "ContactDetectionPlugin estimating contact from AnymalState topic "
            << anymal_state_topic
            << " with GRF threshold: "
            << grf_threshold_
        );
    }

    void anymalStateCallback(const anymal_msgs::AnymalState::ConstPtr& anymal_state)
    {
        if (!estimator_configured_)
        {
            ROS_WARN_STREAM_THROTTLE(
                1.0,
                "ContactDetectionPlugin received AnymalState, but GRF estimator is not configured"
            );
            return;
        }

        const ros::Time stamp = stampFromAnymalState(*anymal_state);

        state_estimator::GrfContactEstimate estimate;

        const state_estimator::JointStateSnapshot joints =
            jointSnapshotFromAnymalState(*anymal_state);

        if (!grf_contact_estimator_.update(joints, estimate))
        {
            ROS_WARN_STREAM_THROTTLE(
                1.0,
                "GRF contact estimator update failed: "
                << grf_contact_estimator_.lastError()
            );
            return;
        }

        ROS_DEBUG_STREAM_THROTTLE(
            1.0,
            "Estimated contact metrics [LF RF LH RH]: "
            << estimate.metric[0] << ", "
            << estimate.metric[1] << ", "
            << estimate.metric[2] << ", "
            << estimate.metric[3]
        );

        publishContact(stamp, estimate.stance);
    }

    void publishContact(
        const ros::Time& stamp,
        const std::array<bool, 4>& stance)
    {
        msg_.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;

        msg_.stance_lf = stance[0];
        msg_.stance_rf = stance[1];
        msg_.stance_lh = stance[2];
        msg_.stance_rh = stance[3];

        pub_.publish(msg_);
    }

    static ros::Time stampFromAnymalState(
        const anymal_msgs::AnymalState& anymal_state)
    {
        if (!anymal_state.header.stamp.isZero())
        {
            return anymal_state.header.stamp;
        }

        if (!anymal_state.joints.header.stamp.isZero())
        {
            return anymal_state.joints.header.stamp;
        }

        if (!anymal_state.pose.header.stamp.isZero())
        {
            return anymal_state.pose.header.stamp;
        }

        if (!anymal_state.twist.header.stamp.isZero())
        {
            return anymal_state.twist.header.stamp;
        }

        return ros::Time::now();
    }

    static state_estimator::JointStateSnapshot jointSnapshotFromAnymalState(
        const anymal_msgs::AnymalState& anymal_state)
    {
        state_estimator::JointStateSnapshot joints;

        joints.name = anymal_state.joints.name;
        joints.position = anymal_state.joints.position;
        joints.velocity = anymal_state.joints.velocity;
        joints.acceleration = anymal_state.joints.acceleration;
        joints.effort = anymal_state.joints.effort;

        return joints;
    }

    static std::string resolveRosFind(const std::string& input)
    {
        std::string resolved = input;

        const std::string find_token = "$(find ";
        const size_t token_pos = resolved.find(find_token);

        if (token_pos == std::string::npos)
        {
            return resolved;
        }

        const size_t package_start = token_pos + find_token.length();
        const size_t package_end = resolved.find(")", package_start);

        if (package_end == std::string::npos)
        {
            return resolved;
        }

        const std::string package_name =
            resolved.substr(package_start, package_end - package_start);

        const std::string package_path = ros::package::getPath(package_name);

        if (package_path.empty())
        {
            return resolved;
        }

        resolved.replace(
            token_pos,
            package_end - token_pos + 1,
            package_path
        );

        return resolved;
    }

    static std::array<std::string, 4> toStringArray4(
        const std::vector<std::string>& values,
        const std::array<std::string, 4>& fallback,
        const std::string& param_name)
    {
        if (values.size() != 4)
        {
            ROS_WARN_STREAM(
                "Parameter "
                << param_name
                << " must contain exactly 4 strings. Using defaults."
            );

            return fallback;
        }

        return {{values[0], values[1], values[2], values[3]}};
    }

    static Eigen::Vector3d toVector3(
        const std::vector<double>& values,
        const Eigen::Vector3d& fallback,
        const std::string& param_name)
    {
        if (values.size() != 3)
        {
            ROS_WARN_STREAM(
                "Parameter "
                << param_name
                << " must contain exactly 3 values. Using default."
            );

            return fallback;
        }

        return Eigen::Vector3d(values[0], values[1], values[2]);
    }

    ros::Subscriber anymal_state_sub_;
    ros::Publisher pub_;

    bool estimator_configured_{false};

    state_estimator::GrfContactEstimator grf_contact_estimator_;
    state_estimator_msgs::ContactDetection msg_;

    double grf_threshold_{30.0};
};

} // namespace state_estimator_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
    state_estimator_plugins::ContactDetectionPlugin,
    state_estimator_plugins::PluginBase
)