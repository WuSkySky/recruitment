// Copyright 2026 Recruitment Simulation Maintainers
// SPDX-License-Identifier: Apache-2.0
#include "Runtime.hh"
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/rendering/Camera.hh>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cstring>
namespace recruitment_sim {
class Sensors : public gazebo::SensorPlugin {
public:
  ~Sensors() override {
    connection_.reset();
    {std::lock_guard<std::mutex> lock(*mutex_); *alive_=false;}
    if(domain_) domain_->Remove(node_);
  }
  void Load(gazebo::sensors::SensorPtr sensor,sdf::ElementPtr sdf) override {
    sensor_=sensor;
    domain_=GetDomain(sdf->Get<unsigned int>("domain"));
    node_=domain_->Node(sensor->Name()+"_sensor",sdf->Get<std::string>("namespace"));
    frame_=sdf->Get<std::string>("frame_id");
    // Reliable publishers remain compatible with both reliable Web/RViz and
    // best-effort sensor subscribers. Keep a small queue for large streams.
    const auto qos=rclcpp::QoS(2).reliable();
    camera_=std::dynamic_pointer_cast<gazebo::sensors::CameraSensor>(sensor);
    imu_=std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(sensor);
    lidar_=std::dynamic_pointer_cast<gazebo::sensors::GpuRaySensor>(sensor);
    if(camera_) {
      image_=node_->create_publisher<sensor_msgs::msg::Image>("camera/image",qos);
      info_=node_->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info",qos);
    } else if(imu_) imu_pub_=node_->create_publisher<sensor_msgs::msg::Imu>("gimbal_imu",qos);
    else if(lidar_) cloud_=node_->create_publisher<sensor_msgs::msg::PointCloud2>("livox/lidar",qos);
    else throw std::runtime_error("unsupported Classic sensor type");
    auto alive=alive_; auto mutex=mutex_;
    connection_=sensor_->ConnectUpdated([this,alive,mutex] {
      std::lock_guard<std::mutex> lock(*mutex); if(*alive) Publish();
    });
    sensor_->SetActive(true);
  }
private:
  void Publish() {
    std_msgs::msg::Header header;
    header.stamp=rclcpp::Time(Nanoseconds(sensor_->LastUpdateTime())); header.frame_id=frame_;
    if(camera_) {
      auto data=camera_->ImageData(); if(!data) return;
      sensor_msgs::msg::Image msg; msg.header=header; msg.width=camera_->ImageWidth();msg.height=camera_->ImageHeight();
      msg.encoding="rgb8";msg.step=msg.width*3;msg.data.assign(data,data+msg.step*msg.height);image_->publish(msg);
      sensor_msgs::msg::CameraInfo info;info.header=header;info.width=msg.width;info.height=msg.height;info.distortion_model="plumb_bob";info.d.assign(5,0);
      const double f=msg.width/(2*std::tan(camera_->Camera()->HFOV().Radian()/2));
      const double cx=(msg.width-1)*.5,cy=(msg.height-1)*.5;
      info.k={f,0,cx,0,f,cy,0,0,1}; info.r={1,0,0,0,1,0,0,0,1};info.p={f,0,cx,0,0,f,cy,0,0,0,1,0};info_->publish(info);
    } else if(imu_) {
      sensor_msgs::msg::Imu msg;msg.header=header;
      auto q=imu_->Orientation();auto w=imu_->AngularVelocity();auto a=imu_->LinearAcceleration();
      msg.orientation.x=q.X();msg.orientation.y=q.Y();msg.orientation.z=q.Z();msg.orientation.w=q.W();
      msg.angular_velocity.x=w.X();msg.angular_velocity.y=w.Y();msg.angular_velocity.z=w.Z();
      msg.linear_acceleration.x=a.X();msg.linear_acceleration.y=a.Y();msg.linear_acceleration.z=a.Z();imu_pub_->publish(msg);
    } else if(lidar_) {
      std::vector<double> ranges;lidar_->Ranges(ranges);
      const int width=lidar_->RangeCount(),height=lidar_->VerticalRangeCount();
      if(ranges.size()!=static_cast<size_t>(width*height)) return;
      sensor_msgs::msg::PointCloud2 msg;msg.header=header;msg.height=height;msg.width=width;msg.is_dense=false;
      sensor_msgs::PointCloud2Modifier modifier(msg);modifier.setPointCloud2Fields(4,"x",1,sensor_msgs::msg::PointField::FLOAT32,"y",1,sensor_msgs::msg::PointField::FLOAT32,"z",1,sensor_msgs::msg::PointField::FLOAT32,"intensity",1,sensor_msgs::msg::PointField::FLOAT32);
      modifier.resize(width*height);
      msg.width=width; msg.height=height; msg.row_step=msg.point_step*width;
      sensor_msgs::PointCloud2Iterator<float> x(msg,"x"),y(msg,"y"),z(msg,"z"),intensity(msg,"intensity");
      for(int row=0;row<height;++row) for(int col=0;col<width;++col,++x,++y,++z,++intensity) {
        const int index=row*width+col;const double r=ranges[index];
        const double yaw=lidar_->AngleMin().Radian()+col*(lidar_->AngleMax()-lidar_->AngleMin()).Radian()/std::max(1,width-1);
        const double pitch=lidar_->VerticalAngleMin().Radian()+row*(lidar_->VerticalAngleMax()-lidar_->VerticalAngleMin()).Radian()/std::max(1,height-1);
        if(!std::isfinite(r) || r<lidar_->RangeMin() || r>lidar_->RangeMax()) *x=*y=*z=std::numeric_limits<float>::quiet_NaN();
        else {*x=r*std::cos(pitch)*std::cos(yaw);*y=r*std::cos(pitch)*std::sin(yaw);*z=r*std::sin(pitch);}
        *intensity=lidar_->Retro(index);
      }
      cloud_->publish(msg);
    }
  }
  gazebo::sensors::SensorPtr sensor_; gazebo::sensors::CameraSensorPtr camera_;
  gazebo::sensors::ImuSensorPtr imu_;gazebo::sensors::GpuRaySensorPtr lidar_;
  gazebo::event::ConnectionPtr connection_;std::shared_ptr<Domain> domain_;rclcpp::Node::SharedPtr node_;
  std::string frame_;std::shared_ptr<bool> alive_{std::make_shared<bool>(true)};
  std::shared_ptr<std::mutex> mutex_{std::make_shared<std::mutex>()};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
};
GZ_REGISTER_SENSOR_PLUGIN(Sensors)
}
