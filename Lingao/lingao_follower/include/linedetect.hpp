/** MIT License
Copyright (c) 2017 Sudarshan Raghunathan
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
*
*@copyright Copyright 2017 Sudarshan Raghunathan
*@file linedetect.hpp
*@author Sudarshan Raghunathan
*@brief Header file for class linedetect
*/

#pragma once
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"

/**
 *@brief Line Detect class contains all the functions for image procesing and direction publishing
 */
class LineDetect {
  public:
    cv::Mat img;  // opencv矩阵格式的输入图像
    cv::Mat img_filt;  // opencv矩阵格式的过滤图像
    cv::Mat img_out;  // opencv矩阵格式的过滤图像
    cv::Mat hsv_out;  // HSV转换后图像
    float target_normalized; // 计算输出归一化方向

  cv::Mat HSV_Filter(const cv::Mat& input_image);
    /**
     *@brief 用于从LingAobot订阅图像主题并转换为opencv图像格式的回调
      *@param msg ROS的图片信息
      *@return none
    */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    /**
     *@brief 在输入图像中应用高斯滤波器的函数
     *@param input opencv矩阵格式的图像
     *@return opencv矩阵格式的高斯滤波后图像
     */
    cv::Mat Gauss(cv::Mat input);
    /**
     *@brief 使用颜色阈值、图像屏蔽和质心检测来执行线检测以发布方向的功能
     *@param input opencv 矩阵格式的过滤后的输入图像
     *@return 返回检测位置归一化值，大于等于1未找到
     */
    float colorthresh(cv::Mat input);

    LineDetect();
    LineDetect(cv::Scalar lower, cv::Scalar upper);
    void SetLineColorScalar(cv::Scalar lower, cv::Scalar upper);

  private:
    cv::Scalar LowerColor;
    cv::Scalar UpperColor;
    cv::Mat img_hsv;
    cv::Mat img_mask;
};
