
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <airplane_gazebo_plugin/airplane_3d_movement_gazebo_plugin.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Quaternion.hh>
#include <geometry_msgs/msg/twist.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>


namespace airplane_3d_movement_gazebo_plugin
{
    class Airplane3DMovementGazeboPluginPrivate
    {
        public:
        void OnUpdate(const gazebo::common::UpdateInfo & _info);
        void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);
        void UpdateOdometry(const gazebo::common::Time & _current_time);
        void PublishOdometryTf(const gazebo::common::Time & _current_time);
        gazebo_ros::Node::SharedPtr ros_node_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
        geometry_msgs::msg::Twist target_cmd_vel_;
        nav_msgs::msg::Odometry odom_;
        gazebo::physics::WorldPtr world_;
        gazebo::physics::ModelPtr model_;
        gazebo::event::ConnectionPtr update_connection_;
        std::mutex lock_;
        double update_period_;
        double publish_period_;
        gazebo::common::Time last_update_time_;
        gazebo::common::Time last_publish_time_;
        std::string odometry_frame_;
        std::string robot_base_frame_;
        bool publish_odom_;
        bool publish_odom_tf_;

    };
    
    Airplane3DMovementGazeboPlugin::Airplane3DMovementGazeboPlugin()
    :impl_(std::make_unique<Airplane3DMovementGazeboPluginPrivate>())
    {
    }

    Airplane3DMovementGazeboPlugin::~Airplane3DMovementGazeboPlugin()
    {
    }

    void Airplane3DMovementGazeboPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        impl_->model_=_model;
        impl_->world_ = _model->GetWorld();
        impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
        const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();


        impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;
        impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;


        auto update_rate = _sdf->Get<double>("update_rate", 20.0).first;
        if (update_rate > 0.0) {
            impl_->update_period_ = 1.0 / update_rate;
        } else {
            impl_->update_period_ = 0.0;
        }
        impl_->last_update_time_ = impl_->world_->SimTime();

        auto publish_rate = _sdf->Get<double>("publish_rate", 20.0).first;
        if (update_rate > 0.0) {
            impl_->publish_period_ = 1.0 / publish_rate;
        } else {
            impl_->publish_period_ = 0.0;
        }
        impl_->last_publish_time_ = impl_->world_->SimTime();

        impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
            std::bind(&Airplane3DMovementGazeboPluginPrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

        RCLCPP_INFO(
            impl_->ros_node_->get_logger(), "Subscribed to [%s]",
            impl_->cmd_vel_sub_->get_topic_name());

        
        impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", true).first;
        if (impl_->publish_odom_) {
            impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
            "odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

        RCLCPP_INFO(
            impl_->ros_node_->get_logger(), "Advertise odometry on [%s]",
            impl_->odometry_pub_->get_topic_name());
        }
        impl_->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", true).first;
        if (impl_->publish_odom_tf_) {
            impl_->transform_broadcaster_ =
            std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

            RCLCPP_INFO(
            impl_->ros_node_->get_logger(),
            "Publishing odom transforms between [%s] and [%s]", impl_->odometry_frame_.c_str(),
            impl_->robot_base_frame_.c_str());
        }
        auto covariance_x = _sdf->Get<double>("covariance_x", 0.00001).first;
        auto covariance_y = _sdf->Get<double>("covariance_y", 0.00001).first;
        auto covariance_z = _sdf->Get<double>("covariance_z", 0.00001).first;
        auto covariance_roll = _sdf->Get<double>("covariance_roll", 0.001).first;
        auto covariance_pitch = _sdf->Get<double>("covariance_pitch", 0.001).first;
        auto covariance_yaw = _sdf->Get<double>("covariance_yaw", 0.001).first;

        // Set covariance
        impl_->odom_.pose.covariance[0] = covariance_x;
        impl_->odom_.pose.covariance[7] = covariance_y;
        impl_->odom_.pose.covariance[14] = covariance_z;
        impl_->odom_.pose.covariance[21] = covariance_roll;
        impl_->odom_.pose.covariance[28] = covariance_pitch;
        impl_->odom_.pose.covariance[35] = covariance_yaw;

        impl_->odom_.twist.covariance[0] = covariance_x;
        impl_->odom_.twist.covariance[7] = covariance_y;
        impl_->odom_.twist.covariance[14] = covariance_z;
        impl_->odom_.twist.covariance[21] = covariance_roll;
        impl_->odom_.twist.covariance[28] = covariance_yaw;
        impl_->odom_.twist.covariance[35] = covariance_yaw;

        // Set header
        impl_->odom_.header.frame_id = impl_->odometry_frame_;
        impl_->odom_.child_frame_id = impl_->robot_base_frame_;

        // Listen to the update event (broadcast every simulation iteration)
        impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&Airplane3DMovementGazeboPluginPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
    }

    void Airplane3DMovementGazeboPlugin::Reset()
    {
        impl_->last_update_time_ = impl_->world_->SimTime();
        impl_->target_cmd_vel_.linear.x = 0;
        impl_->target_cmd_vel_.linear.y = 0;
        impl_->target_cmd_vel_.linear.z = 0;
        impl_->target_cmd_vel_.angular.x = 0;
        impl_->target_cmd_vel_.angular.y = 0;
        impl_->target_cmd_vel_.angular.z = 0;
    }

    void Airplane3DMovementGazeboPluginPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
    {
        double seconds_since_last_update=(_info.simTime - last_update_time_).Double();
    #ifdef IGN_PROFILER_ENABLE
        IGN_PROFILE("Airplane3DMovementGazeboPluginPrivate::OnUpdate");
        IGN_PROFILE_BEGIN("fill ROS message");
    #endif
        if (seconds_since_last_update>=update_period_){
            ignition::math::Pose3d pose=model_->WorldPose();
            ignition::math::Quaterniond orientation = pose.Rot();
            auto yaw = static_cast<float>(pose.Rot().Yaw());
            auto pitch = static_cast<float>(pose.Rot().Pitch());
            ignition::math::Vector3d localVelocity(target_cmd_vel_.linear.x,target_cmd_vel_.linear.y,target_cmd_vel_.linear.z);
            ignition::math::Vector3d worldVelocity = orientation.RotateVector(localVelocity);
            model_->SetLinearVel(worldVelocity);
            model_->SetAngularVel(
                ignition::math::Vector3d(
                    -target_cmd_vel_.angular.y*sinf(yaw)+target_cmd_vel_.angular.x*cosf(yaw)*cosf(pitch),
                    target_cmd_vel_.angular.y*cosf(yaw)+target_cmd_vel_.angular.x*sinf(yaw),
                    target_cmd_vel_.angular.z-target_cmd_vel_.angular.x*cosf(yaw)*sinf(pitch)));
            last_update_time_=_info.simTime;
        }
    #ifdef IGN_PROFILER_ENABLEmodel
        IGN_PROFILE_END();
    #endif
        if (publish_odom_ || publish_odom_tf_) {
            double seconds_since_last_publish = (_info.simTime - last_publish_time_).Double();
            if (seconds_since_last_publish < publish_period_) {
            return;
            }
    #ifdef IGN_PROFILER_ENABLE
        IGN_PROFILE_BEGIN("UpdateOdometry");
    #endif
        UpdateOdometry(_info.simTime);
    #ifdef IGN_PROFILER_ENABLE
        IGN_PROFILE_END();
    #endif
        if (publish_odom_) {
    #ifdef IGN_PROFILER_ENABLE
        IGN_PROFILE_BEGIN("publish odometry");
    #endif
        odometry_pub_->publish(odom_);
    #ifdef IGN_PROFILER_ENABLE
        IGN_PROFILE_END();
    #endif
        }
        if (publish_odom_tf_) {
    #ifdef IGN_PROFILER_ENABLE
        IGN_PROFILE_BEGIN("publish odometryTF");
    #endif
        PublishOdometryTf(_info.simTime);
    #ifdef IGN_PROFILER_ENABLE
        IGN_PROFILE_END();
    #endif
        }

        last_publish_time_ = _info.simTime;
        }

    }

    void Airplane3DMovementGazeboPluginPrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
    {
        std::lock_guard<std::mutex> scoped_lock(lock_);
        target_cmd_vel_ = *_msg;
    }

    void Airplane3DMovementGazeboPluginPrivate::UpdateOdometry(const gazebo::common::Time & _current_time)
    {
        auto pose = model_->WorldPose();
        odom_.pose.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(pose);


        // Convert velocity to child_frame_id(aka base_footprint)
        auto linear = model_->WorldLinearVel();
        auto angular = model_->WorldAngularVel();
        // Linear velocity
        odom_.twist.twist.linear.x = linear.X();
        odom_.twist.twist.linear.y = linear.Y();
        odom_.twist.twist.linear.z = linear.Z();
        //Angular Velocity
        odom_.twist.twist.angular.x = angular.X();
        odom_.twist.twist.angular.y = angular.Y();
        odom_.twist.twist.angular.z = angular.Z();


        // Set timestamp
        odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
    }

    void Airplane3DMovementGazeboPluginPrivate::PublishOdometryTf(const gazebo::common::Time & _current_time)
    {
        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
        msg.header.frame_id = odometry_frame_;
        msg.child_frame_id = robot_base_frame_;
        msg.transform = gazebo_ros::Convert<geometry_msgs::msg::Transform>(odom_.pose.pose);

        transform_broadcaster_->sendTransform(msg);
    }


    GZ_REGISTER_MODEL_PLUGIN(Airplane3DMovementGazeboPlugin)
}  // namespace gazebo_plugins





