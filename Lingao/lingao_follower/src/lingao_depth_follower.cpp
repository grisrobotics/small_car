/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
// #include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>

#include "dynamic_reconfigure/server.h"
#include <lingao_follower/FollowerConfig.h>

#include <depth_image_proc/depth_traits.h>



/**
 * The lingaobot follower nodelet. Subscribes to point clouds
 * from the 3dsensor, processes them, and publishes command vel
 * messages.
 */
class LingaobotFollower
{
public:
  /*!
   * @brief The constructor for the follower.
   * Constructor for the follower.
   */
  LingaobotFollower(ros::NodeHandle* nh, ros::NodeHandle* pnh) : min_y_(0.1), max_y_(0.5),
                        min_x_(-0.2), max_x_(0.2),
                        max_z_(0.8), goal_z_(0.6),
                        z_scale_(1.0), x_scale_(5.0)
  {
    //从参数服务器获取设置的参数（launch文件中设置数值）
      pnh->getParam("min_y", min_y_);
      pnh->getParam("max_y", max_y_);
      pnh->getParam("min_x", min_x_);
      pnh->getParam("max_x", max_x_);
      pnh->getParam("max_z", max_z_);
      pnh->getParam("goal_z", goal_z_);
      pnh->getParam("z_scale", z_scale_);
      pnh->getParam("x_scale", x_scale_);
      pnh->getParam("enabled", enabled_);

      cmdpub_ = nh->advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
      markerpub_ = pnh->advertise<visualization_msgs::Marker>("marker",1);
      bboxpub_ = pnh->advertise<visualization_msgs::Marker>("bbox",1);
      sub_= nh->subscribe<sensor_msgs::Image>("/camera/depth/image_rect", 1, &LingaobotFollower::imagecb, this);

      config_srv_ = new dynamic_reconfigure::Server<lingao_follower::FollowerConfig>(*pnh);
      dynamic_reconfigure::Server<lingao_follower::FollowerConfig>::CallbackType f =
          boost::bind(&LingaobotFollower::reconfigure, this, _1, _2);
      config_srv_->setCallback(f);
  }

  ~LingaobotFollower()
  {
    delete config_srv_;
  }

private:
    double min_y_; /**< The minimum y position of the points in the box. */
  double max_y_; /**< The maximum y position of the points in the box. */
  double min_x_; /**< The minimum x position of the points in the box. */
  double max_x_; /**< The maximum x position of the points in the box. */
  double max_z_; /**< The maximum z position of the points in the box. 框中 点的最大z位置，以上四个字段用来设置框的大小*/
  double goal_z_; /**< The distance away from the robot to hold the centroid 离机器人的距离，以保持质心*/
  double z_scale_; /**< The scaling factor for translational robot speed 移动机器人速度的缩放系数*/
  double x_scale_; /**< The scaling factor for rotational robot speed 旋转机器人速度的缩放系数*/
  bool   enabled_; /**< Enable/disable following; just prevents motor commands 启用/禁用追踪; 只是阻止电机命令，置为false后，机器人不会移动*/

  // 动态配置服务
  dynamic_reconfigure::Server<lingao_follower::FollowerConfig>* config_srv_;

  /*!
   * @brief OnInit method from node handle.
   * OnInit method from node handle. Sets up the parameters
   * and topics.
   */
  // virtual void onInit()
  // {
  //   ros::NodeHandle& nh = getNodeHandle();
  //   ros::NodeHandle& pnh_ = getPrivateNodeHandle();

  //   pnh_.getParam("min_y", min_y_);
  //   pnh_.getParam("max_y", max_y_);
  //   pnh_.getParam("min_x", min_x_);
  //   pnh_.getParam("max_x", max_x_);
  //   pnh_.getParam("max_z", max_z_);
  //   pnh_.getParam("goal_z", goal_z_);
  //   pnh_.getParam("z_scale", z_scale_);
  //   pnh_.getParam("x_scale", x_scale_);
  //   pnh_.getParam("enabled", enabled_);

  //   cmdpub_ = pnh_.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
  //   markerpub_ = pnh_.advertise<visualization_msgs::Marker>("marker",1);
  //   bboxpub_ = pnh_.advertise<visualization_msgs::Marker>("bbox",1);
  //   sub_= nh.subscribe<sensor_msgs::Image>("/camera/depth/image_rect", 1, &LingaobotFollower::imagecb, this);

  //   config_srv_ = new dynamic_reconfigure::Server<lingao_follower::FollowerConfig>(pnh_);
  //   dynamic_reconfigure::Server<lingao_follower::FollowerConfig>::CallbackType f =
  //       boost::bind(&LingaobotFollower::reconfigure, this, _1, _2);
  //   config_srv_->setCallback(f);
  // }

  void reconfigure(lingao_follower::FollowerConfig &config, uint32_t level)
  {
    min_y_ = config.min_y;
    max_y_ = config.max_y;
    min_x_ = config.min_x;
    max_x_ = config.max_x;
    max_z_ = config.max_z;
    goal_z_ = config.goal_z;
    z_scale_ = config.z_scale;
    x_scale_ = config.x_scale;
  }

  /*!
   * @brief Callback for point clouds. 深度点云接收
   * Callback for depth images. It finds the centroid
   * of the points in a box in the center of the image. 
   * Publishes cmd_vel messages with the goal from the image.
   * @param cloud The point cloud message.
   */
  void imagecb(const sensor_msgs::ImageConstPtr& depth_msg)
  {

    // Precompute the sin function for each row and column 预计算每行每列的正弦函数
    uint32_t image_width = depth_msg->width;
    float x_radians_per_pixel = 60.0/57.0/image_width;  // 每个像素的弧度
    float sin_pixel_x[image_width];
    for (int x = 0; x < image_width; ++x) {
      //求出正弦值
      sin_pixel_x[x] = sin((x - image_width/ 2.0)  * x_radians_per_pixel);
    }

    uint32_t image_height = depth_msg->height;
    float y_radians_per_pixel = 45.0/57.0/image_width;
    float sin_pixel_y[image_height];
    for (int y = 0; y < image_height; ++y) {
      // Sign opposite x for y up values
      sin_pixel_y[y] = sin((image_height/ 2.0 - y)  * y_radians_per_pixel);
    }

    //X,Y,Z of the centroid xyz的质心
    float x = 0.0;
    float y = 0.0;
    float z = 1e6;
    //Number of points observed 观察的点数
    unsigned int n = 0;

    //Iterate through all the points in the region and find the average of the position 迭代通过该区域的所有点，找到位置的平均值
    const float* depth_row = reinterpret_cast<const float*>(&depth_msg->data[0]);
    int row_step = depth_msg->step / sizeof(float);
    for (int v = 0; v < (int)depth_msg->height; ++v, depth_row += row_step)
    {
     for (int u = 0; u < (int)depth_msg->width; ++u)
     {
       float depth = depth_image_proc::DepthTraits<float>::toMeters(depth_row[u]);
       if (!depth_image_proc::DepthTraits<float>::valid(depth) || depth > max_z_) continue; //跳过无效的深度值或者深度超出max_z_
       float y_val = sin_pixel_y[v] * depth;
       float x_val = sin_pixel_x[u] * depth;
       if ( y_val > min_y_ && y_val < max_y_ &&
            x_val > min_x_ && x_val < max_x_)
       {
         x += x_val;
         y += y_val;
         z = std::min(z, depth); //approximate depth as forward.
         n++;
       }
     }
    }

    //If there are points, find the centroid and calculate the command goal.
    //If there are no points, simply publish a stop goal.
    //如果有点，找到质心并计算命令目标。如果没有点，只需发布停止消息。
    if (n>4000)
    {
      x /= n;
      y /= n;
      if(z > max_z_){
        ROS_INFO_THROTTLE(1, "Centroid too far away %f, stopping the robot\n", z);
        if (enabled_)
        {
          cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
        }
        return;
      }

      ROS_INFO_THROTTLE(1, "Centroid at %f %f %f with %d points", x, y, z, n);
      publishMarker(x, y, z);

      if (enabled_)
      {
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
        cmd->linear.x = (z - goal_z_) * z_scale_;
        cmd->angular.z = -x * x_scale_;
        cmdpub_.publish(cmd);
        // ROS_INFO("Subscribe to the linear.x %0.4f, anz %0.4f", cmd->linear.x, cmd->angular.z);
      }
    }
    else
    {
      ROS_INFO_THROTTLE(1, "Not enough points(%d) detected, stopping the robot", n);
      publishMarker(x, y, z);

      if (enabled_)
      {
        cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      }
    }

    publishBbox();
  }


  void publishMarker(double x,double y,double z)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    markerpub_.publish( marker );
  }

  void publishBbox()
  {
    double x = (min_x_ + max_x_)/2;
    double y = (min_y_ + max_y_)/2;
    double z = (0 + max_z_)/2;

    double scale_x = (max_x_ - x)*2;
    double scale_y = (max_y_ - y)*2;
    double scale_z = (max_z_ - z)*2;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = -y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = scale_z;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    bboxpub_.publish( marker );
  }

  ros::Subscriber sub_;
  ros::Publisher cmdpub_;
  ros::Publisher markerpub_;
  ros::Publisher bboxpub_;
};

int main(int argc, char** argv)
{
    // 初始化节点
    ros::init(argc, argv, "lingao_depth_follow_node");
    ros::NodeHandle pnh_("~");
    ros::NodeHandle nh_;

    LingaobotFollower lingao_deph_follow(&nh_, &pnh_);
    ros::spin();

    return 0;
}
