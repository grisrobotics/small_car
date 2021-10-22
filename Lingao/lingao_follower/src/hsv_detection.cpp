/*
 * Copyright (C) 2021, LingAo Robotics, INC.
 * @Version V1.0
 * @Author owen
 * @Date 2021-09-05 21:19:09
 * @LastEditTime 2021-09-11 22:01:46
 * @LastEditors owen
 * @Description
 * @FilePath /lingao_ws/src/lingaoRobot/lingao_visual_apps/lingao_follower/src/hsv_detection.cpp
 */

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <lingao_follower/HSV_reconfigureConfig.h>
#include "linedetect.hpp"

LineDetect line_detect;
cv::Scalar lower_color_range_, upper_color_range_;

// 动态参数调节回调
void dyn_callback(lingao_follower::HSV_reconfigureConfig& config, uint32_t level)
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
}

int main(int argc, char** argv)
{
    // 初始化节点
    ros::init(argc, argv, "hsv_detection");
    ros::NodeHandle nh_;

    //动态参数调节
    dynamic_reconfigure::Server<lingao_follower::HSV_reconfigureConfig> dyn_server;
    dynamic_reconfigure::Server<lingao_follower::HSV_reconfigureConfig>::CallbackType dyn_f;
    dyn_f = boost::bind(&dyn_callback, _1, _2);
    dyn_server.setCallback(dyn_f);

    // 订阅摄像头发出主题
    ros::Subscriber sub = nh_.subscribe("/camera/rgb/image_raw", 1, &LineDetect::imageCallback, &line_detect);
    //发布ROS输出图像
    image_transport::Publisher img_pub_ =  image_transport::ImageTransport(nh_).advertise("hsv_detection", 1);
    //发布HSV输出图像
    // image_transport::Publisher hsv_pub_ =  image_transport::ImageTransport(nh_).advertise("hsv_filter", 1);

    while (ros::ok())
    {
        if (!line_detect.img.empty())
        {
            // 执行图像处理
            line_detect.img_filt = line_detect.Gauss(line_detect.img);
            line_detect.colorthresh(line_detect.img_filt);
            try
            {
                //将CV图像转成ROS图像
                sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, line_detect.img_out).toImageMsg();

                // cv::Mat out_frame = line_detect.HSV_Filter(line_detect.img_filt );
                // sensor_msgs::Image::Ptr out_hsv_img = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, out_frame).toImageMsg();
                //发布图片视频到ROS
                img_pub_.publish(out_img);
                // hsv_pub_.publish(out_hsv_img);
            }
            catch(cv::Exception& e)
            {
               ROS_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
            }
            

        }
        ros::spinOnce();
    }

}
