#include "odometry_private.hpp"

#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/components/AngularAcceleration.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>

namespace odometry {

void PluginPrivate::ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf) {
  sdf_params_.link = _sdf->Get<std::string>("link", sdf_params_.link).first;

  sdf_params_.base_topic =
      _sdf->Get<std::string>("base_topic", sdf_params_.base_topic).first;

  sdf_params_.update_rate =
      _sdf->Get<double>("update_rate", sdf_params_.update_rate).first;
  if (sdf_params_.update_rate > 0.0) {
    std::chrono::duration<double> period{1.0 / sdf_params_.update_rate};
    update_period_ =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  }

  sdf_params_.angular_velocity_update_rate =
      _sdf->Get<double>("angular_velocity_update_rate",
                        sdf_params_.angular_velocity_update_rate)
          .first;
  if (sdf_params_.update_rate > 0.0) {
    std::chrono::duration<double> period{
        1.0 / sdf_params_.angular_velocity_update_rate};
    angular_velocity_update_period_ =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  }
}

bool PluginPrivate::InitModel(ignition::gazebo::EntityComponentManager &_ecm,
                              ignition::gazebo::Entity _entity) {
  model_ = ignition::gazebo::Model(_entity);
  if (!model_.Valid(_ecm)) {
    return false;
  }
  model_name_ = model_.Name(_ecm);
  InitComponents(_ecm);
  InitHeader();
  return true;
}

void PluginPrivate::Publish(
    const ignition::gazebo::EntityComponentManager &_ecm,
    const ignition::msgs::Time &stamp) {
  msg_.mutable_header()->mutable_stamp()->CopyFrom(stamp);

  auto pose = link_.WorldPose(_ecm);
  auto v_linear = link_.WorldLinearVelocity(_ecm);
  auto v_linear_local = pose->Rot().Inverse().RotateVector(*v_linear);
  auto v_angular = link_.WorldAngularVelocity(_ecm);
  auto v_angular_local = pose->Rot().Inverse().RotateVector(*v_angular);
  auto header = msg_.mutable_header();
  auto twist = msg_.mutable_twist();
  header->mutable_stamp()->CopyFrom(stamp);

  ignition::msgs::Set(msg_.mutable_pose(), *pose);
  ignition::msgs::Set(twist->mutable_angular(), v_angular_local);
  ignition::msgs::Set(twist->mutable_linear(), v_linear_local);
  odometry_pub_.Publish(msg_);
  PublishWorldLinearAcceleration(_ecm, stamp);
}

void PluginPrivate::PublishWorldLinearAcceleration(
    const ignition::gazebo::EntityComponentManager &_ecm,
    const ignition::msgs::Time &_stamp) {
  ignition::msgs::Vector3d msg;

  // set stamp and frame_id in header
  auto header = msg.mutable_header();
  header->mutable_stamp()->CopyFrom(_stamp);
  auto frame = header->add_data();
  frame->set_key("frame_id");
  frame->add_value("map");

  // set linear acceleration vector
  auto linear_acceleration = link_.WorldLinearAcceleration(_ecm).value_or(
      ignition::math::Vector3d::Zero);
  auto pose = link_.WorldPose(_ecm);
  auto linear_acceleration_local = pose->Rot().Inverse().RotateVector(linear_acceleration);
  msg.set_x(linear_acceleration_local.X());
  msg.set_y(linear_acceleration_local.Y());
  msg.set_z(linear_acceleration_local.Z());

  world_linear_acceleration_pub_.Publish(msg);
}



void PluginPrivate::PublishAcceleration(
        const ignition::gazebo::EntityComponentManager &_ecm,
        const std::chrono::steady_clock::duration &_sim_time) {
  auto dt = _sim_time - last_angular_velocity_pub_time_;
  if (dt > std::chrono::steady_clock::duration::zero() &&
      dt < angular_velocity_update_period_) {
    return;
  }
  last_angular_velocity_pub_time_ = _sim_time;

  ignition::msgs::Twist msg;

  auto header = msg.mutable_header();
  auto stamp = ignition::gazebo::convert<ignition::msgs::Time>(_sim_time);
  header->mutable_stamp()->CopyFrom(stamp);
  auto frame = header->add_data();
  frame->set_key("frame_id");
  frame->add_value(model_name_ + "/base_link");

  auto pose = link_.WorldPose(_ecm);
  auto linear_acceleration = link_.WorldLinearAcceleration(_ecm).value_or(
          ignition::math::Vector3d::Zero);
  auto linear_acceleration_local = pose->Rot().Inverse().RotateVector(linear_acceleration);
  auto angular_acceleration = link_.WorldAngularAcceleration(_ecm);
  auto angular_acceleration_local =
          pose->Rot().Inverse().RotateVector(*angular_acceleration);
  ignition::msgs::Set(msg.mutable_linear(), linear_acceleration_local);
  ignition::msgs::Set(msg.mutable_angular(), angular_acceleration_local);
  accelerations_pub_.Publish(msg);
}


void PluginPrivate::Advertise() {
  odometry_pub_ =
      node_.Advertise<ignition::msgs::Odometry>(OdometryTopicName());
  world_linear_acceleration_pub_ =
      node_.Advertise<ignition::msgs::Vector3d>(WorldLinearAccelerationTopicName());
  accelerations_pub_ =
      node_.Advertise<ignition::msgs::Twist>(AccelerationsTopicName());
  }

std::string PluginPrivate::OdometryTopicName() {
  return "/" + model_name_ + "/" + sdf_params_.base_topic;
}

std::string PluginPrivate::WorldLinearAccelerationTopicName() {
  return "/" + model_name_ + "/ground_truth/world_linear_acceleration";
}
std::string PluginPrivate::AccelerationsTopicName() {
  return "/" + model_name_ + "/ground_truth/accelerations";
}

void PluginPrivate::InitHeader() {
  auto header = msg_.mutable_header();
  auto frame = header->add_data();
  frame->set_key("frame_id");
  frame->add_value("map");

  auto child_frame = header->add_data();
  child_frame->set_key("child_frame_id");
  child_frame->add_value(model_name_ + "/base_link");
}

void PluginPrivate::InitComponents(
    ignition::gazebo::EntityComponentManager &_ecm) {
  link_ = ignition::gazebo::Link(model_.LinkByName(_ecm, sdf_params_.link));

  // create component for world pose
  if (!_ecm.Component<ignition::gazebo::components::WorldPose>(
          link_.Entity())) {
    _ecm.CreateComponent(link_.Entity(),
                         ignition::gazebo::components::WorldPose());
  }

    link_.EnableVelocityChecks(_ecm, true);


    // create component for angular velocity
  if (!_ecm.Component<ignition::gazebo::components::AngularVelocity>(
          link_.Entity())) {
    _ecm.CreateComponent(link_.Entity(),
                         ignition::gazebo::components::AngularVelocity());
  }

  // create component for linear velocity
  if (!_ecm.Component<ignition::gazebo::components::LinearVelocity>(
          link_.Entity())) {
    _ecm.CreateComponent(link_.Entity(),
                         ignition::gazebo::components::LinearVelocity());
  }

  // create component for linear acceleration
  if (!_ecm.Component<ignition::gazebo::components::LinearAcceleration>(
          link_.Entity())) {
    _ecm.CreateComponent(link_.Entity(),
                         ignition::gazebo::components::LinearAcceleration());
  }
  link_.EnableAccelerationChecks(_ecm, true);

  // create component for angular acceleration
  if (!_ecm.Component<ignition::gazebo::components::AngularAcceleration>(
          link_.Entity())) {
    _ecm.CreateComponent(link_.Entity(),
                         ignition::gazebo::components::AngularAcceleration());
  }
}
}  // namespace odometry
