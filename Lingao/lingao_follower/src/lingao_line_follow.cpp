/*
 * Copyright (C) 2021, LingAo Robotics, INC.
 * @Version V1.0
 * @Author owen
 * @Date 2021-09-06 19:29:18
 * @LastEditTime 2021-09-12 09:35:29
 * @LastEditors owen
 * @Description
 * @FilePath /lingao_ws/src/lingaoRobot/lingao_visual_apps/lingao_follower/src/lingao_line_follow.cpp
 */

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <lingao_follower/HSV_reconfigureConfig.h>
#include <geometry_msgs/Twist.h>
#include "linedetect.hpp"
#include "lingao_pid.h"

class LineFollow {
  private:
    ros::Subscriber img_sub_;
    ros::Publisher pub_cmd_vel_;
    image_transport::Publisher img_pub_;

    //动态参数调节回调
    dynamic_reconfigure::Server<lingao_follower::HSV_reconfigureConfig> dyn_server;
    dynamic_reconfigure::Server<lingao_follower::HSV_reconfigureConfig>::CallbackType dyn_f;

    LineDetect line_detect;
    cv::Scalar lower_color_range_, upper_color_range_;
    double speed_max;
    bool robot_run;

    void dyn_callback(lingao_follower::HSV_reconfigureConfig& config, uint32_t level);

  public:
    LineFollow(ros::NodeHandle* nh_);
    ~LineFollow();

    void Loop();
};

// 初始化各数据
LineFollow::LineFollow(ros::NodeHandle* nh_)
{
    std::string topic_camera_;
    std::string public_cmd_vel_;

    nh_->param("topic_camera", topic_camera_, std::string("/camera/rgb/image_raw"));
    nh_->param("public_cmd_vel", public_cmd_vel_, std::string("/cmd_vel"));

    // 订阅摄像头发出主题
    ROS_INFO_STREAM("Subscribe to the cmd topic on [" << topic_camera_ << "]");
    img_sub_ = nh_->subscribe(topic_camera_, 1, &LineDetect::imageCallback, &line_detect);

    //发布ROS输出图像
    ROS_INFO_STREAM("Publish to the cmd topic on /hsv_detection");
    img_pub_ = image_transport::ImageTransport(*nh_).advertise("hsv_detection", 1);

    //发布速度控制
    ROS_INFO_STREAM("Publish to the cmd topic on [" << public_cmd_vel_ << "]");
    pub_cmd_vel_ = nh_->advertise<geometry_msgs::Twist>(public_cmd_vel_, 50);

    //动态调参
    dyn_f = boost::bind(&LineFollow::dyn_callback, this, _1, _2);
    dyn_server.setCallback(dyn_f);

    //设置HSV 色域
    lingao_follower::HSV_reconfigureConfig RGBconfig;
    nh_->param("H_Max", RGBconfig.h_limit_max, 255);
    nh_->param("H_Min", RGBconfig.h_limit_min, 100);
    nh_->param("S_Max", RGBconfig.s_limit_max, 255);
    nh_->param("S_Min", RGBconfig.s_limit_min, 100);
    nh_->param("V_Max", RGBconfig.v_limit_max, 30);
    nh_->param("V_Min", RGBconfig.v_limit_min, 20);
    nh_->param("speed", RGBconfig.speed, 0.25);
    nh_->param("robot_run", RGBconfig.Robot_RUN, false);
    dyn_callback(RGBconfig, 0);
}

LineFollow::~LineFollow() {}

// 动态参数调节回调
void LineFollow::dyn_callback(lingao_follower::HSV_reconfigureConfig& config, uint32_t level)
{
    int h_min_  = config.h_limit_min;
    int h_max_ = config.h_limit_max;
    int s_min_  = config.s_limit_min;
    int s_max_ = config.s_limit_max;
    int v_min_  = config.v_limit_min;  
    int v_max_ = config.v_limit_max;

    ROS_INFO("Set HSV space: H:[%d - %d] S:[%d - %d] V:[%d - %d]", h_max_, h_min_, s_max_, s_min_, v_max_, v_min_);

    if (s_max_ < s_min_)
      std::swap(s_max_, s_min_);
    if (v_max_ < v_min_)
      std::swap(v_max_, v_min_);
    lower_color_range_ = cv::Scalar(h_min_ / 2, s_min_, v_min_, 0);
    upper_color_range_ = cv::Scalar(h_max_ / 2, s_max_, v_max_, 0);

    line_detect.SetLineColorScalar(lower_color_range_, upper_color_range_);

    speed_max = config.speed;
    robot_run = config.Robot_RUN;
    if (robot_run)
    {
        ROS_INFO("robot start running!");
    }
}

//循环运行节点
void LineFollow::Loop()
{
    //使用PID 调节yaw方向
    Lingao_PosPID posPID(1.2, 0.05, 0.01, -0.1, 0.1);
    geometry_msgs::Twist cmd_vel;
    double linear_x = 0, ang_z = 0;

    ros::Rate loop_rate(50); // HZ max
    while (ros::ok())
    {
        if (!line_detect.img.empty())
        {
            // 执行图像处理
            line_detect.img_filt = line_detect.Gauss(line_detect.img);
            line_detect.colorthresh(line_detect.img_filt);
            //将CV图像转成ROS图像
            sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, line_detect.img_out).toImageMsg();
            //发布图片视频到ROS
            img_pub_.publish(out_img);

            if (robot_run && line_detect.target_normalized < 1 && line_detect.target_normalized > -1)
            {
                // PID增量控制
                ang_z = posPID.calculation(line_detect.target_normalized, 0) * -1;

                // 限幅
                ang_z    = ang_z >= 1.0 ? 1.0 : ang_z;
                ang_z    = ang_z < -1.0 ? -1.0 : ang_z;
                // 比例角速度限制最大速度
                linear_x = speed_max * (1.0 - (0.9 * fabs(line_detect.target_normalized)));
                
                // Debug
                ROS_INFO("target_nor:[%.3f ] ang_z:[%.3f ] linear_x:[%.4f ]", line_detect.target_normalized, ang_z, linear_x);
            }
            else
            {
                linear_x = 0;
                ang_z    = 0;
            }

            cmd_vel.linear.x  = linear_x;
            cmd_vel.angular.z = ang_z;
            pub_cmd_vel_.publish(cmd_vel);
        }

        ros::spinOnce();
        loop_rate.sleep();  // 等待loop_rate設定的時間
    }
}

int main(int argc, char** argv)
{
    // 初始化节点
    ros::init(argc, argv, "lingao_line_follow_node");
    ros::NodeHandle pnh_("~");

    LineFollow lingao_line_follow(&pnh_);
    lingao_line_follow.Loop();

    return 0;
}