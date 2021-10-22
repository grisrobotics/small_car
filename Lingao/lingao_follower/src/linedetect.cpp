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
*@file linedetect.cpp
*@author Sudarshan Raghunathan
*@brief  Class linedetect's function definitions
*/
#include "linedetect.hpp"
#include <cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "ros/console.h"

// 初始化
LineDetect::LineDetect()
{
    LowerColor = {20, 100, 100}; // HSV 值下限
    UpperColor = {30, 255, 255}; // HSV值上限
}

LineDetect::LineDetect(cv::Scalar lower, cv::Scalar upper)
{
    SetLineColorScalar(lower, upper);
}

// 设置HSV参数
void LineDetect::SetLineColorScalar(cv::Scalar lower, cv::Scalar upper)
{
    LowerColor = lower;
    UpperColor = upper;
}

// 在 OpenCV 和 ROS 之间创建一个CV bridge
void LineDetect::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    //输入数据被复制并转换为BGR8图像，然后保存在“img”中
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img    = cv_ptr->image;
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

// 应用高斯滤波器
cv::Mat LineDetect::Gauss(cv::Mat input)
{
    cv::Mat output;
    // 高斯滤波器
    cv::GaussianBlur(input, output, cv::Size(3, 3), 0.1, 0.1);
    return output;
}

// HSV 滤波器
cv::Mat LineDetect::HSV_Filter(const cv::Mat& input_image)
{
    cv::Mat hls_image, output_image;
    cv::cvtColor(input_image, hls_image, cv::COLOR_BGR2HLS);
    if (LowerColor[0] < UpperColor[0])
    {
      cv::inRange(hls_image, LowerColor, UpperColor, output_image);
    }
    return output_image;
}

/**
 * 功能：限制应用于定义可被编目作为追踪线的颜色的颜色的可能范围。
 * 一旦定义了线，就检测其轮廓并计算线的质心检测
In the following function, limits are applied to define the possible range of colors that can be cataloged as the color of the tracking line.

*/
float LineDetect::colorthresh(cv::Mat input)
{
    // 初始化变量
    cv::Size s = input.size(); // Size of image
    std::vector<std::vector<cv::Point>> v;
    auto w   = s.width;
    auto h   = s.height;
    auto c_x = 0.0;

    // 检测 HSV 范围内的所有对象
    cv::cvtColor(input, LineDetect::img_hsv, CV_BGR2HSV);

    if (LowerColor[0] < UpperColor[0])
    {
      cv::inRange(LineDetect::img_hsv, LowerColor, UpperColor, LineDetect::img_mask);
    }
    img_mask(cv::Rect(0, 0, w, 0.8 * h)) = 0;

    //查找轮廓以获得更好的可视化
    cv::findContours(LineDetect::img_mask, v, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    // 如果存在轮廓，则添加边界
    // 选择面积最大的等高线
    if (v.size() != 0)
    {
        auto area  = 0;
        auto idx   = 0;
        auto count = 0;
        while (count < v.size())
        {
            if (area < v[count].size())
            {
                idx  = count;
                area = v[count].size();
            }
            count++;
        }
        cv::Rect rect = boundingRect(v[idx]);
        
        cv::Point pt1, pt2, pt3;
        pt1.x = rect.x;
        pt1.y = rect.y;
        pt2.x = rect.x + rect.width;
        pt2.y = rect.y + rect.height -5;
        pt3.x = pt1.x + 5;
        pt3.y = pt1.y - 10;
        // 绘制检测结果矩形
        rectangle(input, pt1, pt2, CV_RGB(0, 255, 0), 2);
        // 插入文本框
        cv::putText(input, "Line Detected", pt3, cv::FONT_HERSHEY_COMPLEX, 1, CV_RGB(0, 255, 0));
        img_out = input;
    }
    else img_out = input;
    // 屏蔽图像以限制影响输出的未来转弯
    img_mask(cv::Rect(0.9 * w, 0, 0.1 * w, h)) = 0;
    img_mask(cv::Rect(0, 0, 0.1 * w, h))       = 0;
    // 执行线的质心检测
    cv::Moments M = cv::moments(LineDetect::img_mask);
    if (M.m00 > 0)
    {
        cv::Point p1(M.m10 / M.m00, M.m01 / M.m00);
        cv::circle(LineDetect::img_mask, p1, 5, cv::Scalar(155, 200, 0), -1);
    }
    c_x = M.m10 / M.m00;
    // 选择宽容方向
    // auto tol   = 15;
    // auto count = cv::countNonZero(img_mask);
    // // 如果质心在图像中心的左侧减去公差，则向左转
    // // 如果质心在图像中心的右侧加上公差，则向右转
    // // 如果质心靠近图像中心，则直行
    // if (c_x < w / 2 - tol)
    // {
    //     LineDetect::dir = 0;
    // }
    // else if (c_x > w / 2 + tol)
    // {
    //     LineDetect::dir = 2;
    // }
    // else
    // {
    //     LineDetect::dir = 1;
    // }
    // // 未检测到行线
    // if (count == 0)
    // {
    //     LineDetect::dir = 3;
    // }
    //  ROS_INFO("mid_p:[%.4f ] c_x:[%.4f ] Center:[%.4f ]", mid_p ,c_x  ,(w / 2 - c_x)/(w / 2));

    target_normalized = (w / 2 - c_x)/(w / 2);

    // 输出图像查看
    // cv::namedWindow("LingAobot View");
    // imshow("LingAobot View", input);
    return target_normalized;
}
