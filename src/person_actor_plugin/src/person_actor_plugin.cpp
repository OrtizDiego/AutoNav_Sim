// Copyright 2026 AutoNav Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Velocity-driven pedestrian actor for Gazebo Classic.
//
// Gazebo actors cannot be moved with /set_entity_state: Actor::Update()
// rewrites the pose from its <script> trajectory every frame, and without a
// script the pose stays at the origin. The supported way to drive an actor
// from outside is a custom trajectory plus SetWorldPose()/SetScriptTime(),
// which is only reachable from C++ (same approach as Gazebo's ActorPlugin).
//
// Interface
//   sub  <ns>/cmd_vel  geometry_msgs/Twist   linear.x = forward speed (m/s),
//                                            angular.z = yaw rate (rad/s)
//   pub  <ns>/odom     nav_msgs/Odometry     ground-truth pose in "world"
//
// SDF parameters (all optional)
//   <walking_animation>  name of the walk <animation>     (default "walking")
//   <running_animation>  name of the run <animation>      (default "running")
//   <run_threshold>      speed (m/s) above which the run clip plays (1.6)
//   <walk_animation_factor> clip seconds per metre for walking   (4.15)
//   <run_animation_factor>  clip seconds per metre for running   (1.71)
//   <height>             z of the hip (root bone) frame          (1.2138)
//   <cmd_timeout>        stop if no command for this long (s)    (0.5)
//
// The animation factors were measured from Gazebo's walk.dae (5.75 s clip,
// 1.38 m of hip travel) and run.dae (7.83 s clip, 4.59 m of hip travel).
// Advancing the clip by distance * factor keeps the feet planted, so the
// stride rate follows the commanded speed instead of sliding.

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Actor.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ignition/math/Pose3.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace person_actor_plugin
{

class PersonActorPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    actor_ = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);
    if (!actor_) {
      gzerr << "PersonActorPlugin must be attached to an <actor>\n";
      return;
    }
    world_ = actor_->GetWorld();

    walk_anim_ = sdf->Get<std::string>("walking_animation", "walking").first;
    run_anim_ = sdf->Get<std::string>("running_animation", "running").first;
    run_threshold_ = sdf->Get<double>("run_threshold", 1.6).first;
    walk_factor_ = sdf->Get<double>("walk_animation_factor", 4.15).first;
    run_factor_ = sdf->Get<double>("run_animation_factor", 1.71).first;
    height_ = sdf->Get<double>("height", 1.2138).first;
    cmd_timeout_ = sdf->Get<double>("cmd_timeout", 0.5).first;

    const auto anims = actor_->SkeletonAnimations();
    if (anims.find(walk_anim_) == anims.end()) {
      gzerr << "PersonActorPlugin: animation '" << walk_anim_ << "' not found\n";
      return;
    }
    has_run_anim_ = anims.find(run_anim_) != anims.end();
    if (!has_run_anim_) {
      gzwarn << "PersonActorPlugin: animation '" << run_anim_
             << "' not found, the walk clip will be used at all speeds\n";
    }

    // Initial planar pose from the <actor><pose>; heading is world yaw.
    const auto pose = actor_->WorldPose();
    x_ = pose.Pos().X();
    y_ = pose.Pos().Y();
    heading_ = pose.Rot().Yaw();

    SetAnimation(walk_anim_);

    ros_node_ = gazebo_ros::Node::Get(sdf);
    cmd_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd_v_ = msg->linear.x;
        cmd_w_ = msg->angular.z;
        last_cmd_time_ = world_->SimTime().Double();
      });
    odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(10));

    last_update_ = world_->SimTime().Double();
    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&PersonActorPlugin::OnUpdate, this, std::placeholders::_1));

    RCLCPP_INFO(
      ros_node_->get_logger(), "PersonActorPlugin driving actor '%s'",
      actor_->GetName().c_str());
  }

  void Reset() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd_v_ = 0.0;
    cmd_w_ = 0.0;
    last_update_ = world_ ? world_->SimTime().Double() : 0.0;
  }

private:
  void SetAnimation(const std::string & name)
  {
    if (name == current_anim_) {
      return;
    }
    auto traj = std::make_shared<gazebo::physics::TrajectoryInfo>();
    traj->type = name;
    traj->duration = 1.0;
    actor_->SetCustomTrajectory(traj);
    current_anim_ = name;
  }

  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    const double now = info.simTime.Double();
    const double dt = now - last_update_;
    if (dt <= 0.0) {
      return;
    }
    last_update_ = now;

    double v;
    double w;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool fresh = (now - last_cmd_time_) <= cmd_timeout_;
      v = fresh ? cmd_v_ : 0.0;
      w = fresh ? cmd_w_ : 0.0;
    }
    // Pedestrians do not walk backwards.
    v = std::max(0.0, v);

    heading_ = std::atan2(std::sin(heading_ + w * dt), std::cos(heading_ + w * dt));
    const double dist = v * dt;
    x_ += dist * std::cos(heading_);
    y_ += dist * std::sin(heading_);

    // Hysteresis avoids flickering between clips around the threshold.
    const bool running = current_anim_ == run_anim_;
    if (has_run_anim_ && !running && v > run_threshold_) {
      SetAnimation(run_anim_);
    } else if (running && v < run_threshold_ - 0.2) {
      SetAnimation(walk_anim_);
    }
    const double factor = (current_anim_ == run_anim_) ? run_factor_ : walk_factor_;

    // The skeleton's root frame is Y-up and faces +Y, hence roll and yaw
    // offsets of pi/2 (same convention as Gazebo's ActorPlugin).
    ignition::math::Pose3d pose(
      x_, y_, height_, M_PI_2, 0.0, heading_ + M_PI_2);
    actor_->SetWorldPose(pose, false, false);
    actor_->SetScriptTime(actor_->ScriptTime() + dist * factor);

    if (now - last_odom_time_ >= 1.0 / 30.0) {
      last_odom_time_ = now;
      PublishOdom(now, v, w);
    }
  }

  void PublishOdom(double now, double v, double w)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp.sec = static_cast<int32_t>(std::floor(now));
    odom.header.stamp.nanosec =
      static_cast<uint32_t>((now - std::floor(now)) * 1e9);
    odom.header.frame_id = "world";
    odom.child_frame_id = actor_->GetName();
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = std::sin(heading_ / 2.0);
    odom.pose.pose.orientation.w = std::cos(heading_ / 2.0);
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = w;
    odom_pub_->publish(odom);
  }

  gazebo::physics::ActorPtr actor_;
  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr update_conn_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::mutex mutex_;
  double cmd_v_{0.0};
  double cmd_w_{0.0};
  double last_cmd_time_{-1e9};

  std::string walk_anim_;
  std::string run_anim_;
  std::string current_anim_;
  bool has_run_anim_{false};
  double run_threshold_{1.6};
  double walk_factor_{4.15};
  double run_factor_{1.71};
  double height_{1.2138};
  double cmd_timeout_{0.5};

  double x_{0.0};
  double y_{0.0};
  double heading_{0.0};
  double last_update_{0.0};
  double last_odom_time_{-1e9};
};

GZ_REGISTER_MODEL_PLUGIN(PersonActorPlugin)

}  // namespace person_actor_plugin
