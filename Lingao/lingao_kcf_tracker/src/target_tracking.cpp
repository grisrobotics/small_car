/*
 * Copyright (C) 2021, LingAo Robotics, INC.
 * @Version V1.0
 * @Author owen
 * @Date 2021-09-24 19:14:43
 * @LastEditTime 2021-10-01 16:44:38
 * @LastEditors owen
 * @Description
 * @FilePath /lingao_ws/src/lingaoRobot/lingao_visual_apps/lingao_kcf_tracker/src/target_tracking.cpp
 */

// ROS Libraries
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>

// OpenCV Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <chrono>
#include "tracking_utility.hpp"
#include "kcftracker.hpp"

#include "lingao_pid.h"

typedef std::chrono::time_point<std::chrono::high_resolution_clock> timer;
typedef std::chrono::duration<float> duration;

#define Max_linear_speed 0.4
#define Min_linear_speed 0.2
#define Min_distance 1.5
#define Max_distance 5.0
#define Max_rotation_speed 0.9
#define ERROR_distance 1.0

#define k_linear_speed 0.8
#define k_angula_speed 5

#define focus_X 580     //camera焦距 fx (pixel)

static const std::string RGB_WINDOW_NAME = "RGB Image window";

static const bool HOG = true;   // 使用hog特征
static const bool FIXEDWINDOW = false; // 使用修正窗口
static const bool MULTISCALE = true; // 使用多尺度
static const bool SILENT = true;    // 不做显示
static const bool LAB = false;  //使用LAB颜色

//define a timer
double get_wall_time()  
{  
    struct timeval time ;  
    if (gettimeofday(&time,NULL))
    {  
        return 0;  
    }  
    return (double)time.tv_sec + (double)time.tv_usec * .000001;  
}  

class KCF_Tracking {

  private:
    cv::Mat rgb_image;
    cv::Mat depth_image;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;

    TrackingUtility tu;
    KCFTracker *tracker = NULL;

    ros::Publisher pub_cmd_vel_;

    void imageCallBack(const sensor_msgs::ImageConstPtr& msg);
    void depthCallBack(const sensor_msgs::ImageConstPtr& msg);

  public:
    cv::Rect roi_result;

    KCF_Tracking(ros::NodeHandle* nh);
    ~KCF_Tracking();

    void loop(void);
};

KCF_Tracking::KCF_Tracking(ros::NodeHandle* nh) : it_(*nh)
{
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 10, &KCF_Tracking::imageCallBack, this);
    depth_sub_ = it_.subscribe("/camera/depth/image_rect_raw", 10, &KCF_Tracking::depthCallBack, this);

    pub_cmd_vel_ = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 50);

    cv::namedWindow(RGB_WINDOW_NAME,1);
    cv::setMouseCallback(RGB_WINDOW_NAME, TrackingUtility::mouseCallback, (void*)&tu);
}

KCF_Tracking::~KCF_Tracking() 
{
    // Closing cv window
    cv::destroyWindow(RGB_WINDOW_NAME);
    if(tracker != NULL)
    {
        delete tracker;
        tracker = NULL;
    }
}

// ROS rgb图像回调
void KCF_Tracking::imageCallBack(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    //输入数据被复制并转换为BGR8图像，然后保存在rgb_image中
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        rgb_image    = cv_ptr->image;
        // cv::waitKey(10);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

// ROS 深度图像回调
void KCF_Tracking::depthCallBack(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    //输入数据被复制并转换为CV图像
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        depth_image    = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.", msg->encoding.c_str());
    }
}

void KCF_Tracking::loop()
{
    char message1[100];
    char message2[100];
    cv::Rect roi(0,0,0,0);
    geometry_msgs::Twist cmd_vel;

    timer trackerStartTime;
    duration trackerTimeDiff ;

    float distance = 0, distance_last = 0;
    int center_x;

    float dcxt;
    float cyt,cyt_last,dcyt ;
    float linear_x = 0, angular_z = 0;

    float target_linear_speed_x ;
    float target_linear_speed_y ;

    //使用PID 调节yaw方向
    Lingao_PosPID posPID(1.3, 0.001, 0.001, -0.1, 0.1);

    double start_time, end_time;

    while (ros::ok())
    {
        // Quit if ESC is pressed
        char c = cv::waitKey(10);
        if(c==27)
        {
            break;
        }

        // 将按键触发值传到 TrackingUtility
        tu.getKey(c);

        //等待rgb图像
        if (!rgb_image.empty())
        {
            switch(tu.getState())
            {
            case TrackingUtility::STATE_IDLE:
                roi = tu.getROI();
                sprintf(message2, "Please select ROI and press g");
                break;

            case TrackingUtility::STATE_INIT:
                ROS_INFO("g pressed, initialize tracker");
                sprintf(message2, "g pressed, initialize tracker");
                roi = tu.getROI();
                tracker = new KCFTracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
                tracker->init(roi, rgb_image);
                tu.startTracker();
                break;

            case TrackingUtility::STATE_ONGOING:
                trackerStartTime  = std::chrono::high_resolution_clock::now();
                //  结果框 update()基于当前帧更新目标位置
                roi_result = tracker->update(rgb_image);
                roi = roi_result;
                // 计算更新时长
                trackerTimeDiff = std::chrono::high_resolution_clock::now() - trackerStartTime;
                sprintf(message2, "Tracking: update time = %.2f ms", trackerTimeDiff.count()*1000.0);
                break;

            case TrackingUtility::STATE_STOP:
                ROS_INFO("s pressed, stop tracker");
                sprintf(message2, "s pressed, stop tracker");
                delete tracker;
                tracker = NULL;
                tu.stopTracker();
                roi = tu.getROI();
                break;
                
            default:
                break;
            };

            // 给中心位置到框的线和圆
            cv::circle(rgb_image, cv::Point(rgb_image.cols/2, rgb_image.rows/2), 5, cv::Scalar(255,0,0), 2, 8);
            if(roi.width != 0)
            {
                cv::circle(rgb_image, cv::Point(roi.x + roi.width/2, roi.y + roi.height/2), 3, cv::Scalar(0,0,255), 1, 8);

                cv::line(rgb_image,  cv::Point(rgb_image.cols/2, rgb_image.rows/2),
                        cv::Point(roi.x + roi.width/2, roi.y + roi.height/2),
                        cv::Scalar(0,255,255));
            }

            //在rgb_image上绘制矩形框
            cv::rectangle(rgb_image, roi, cv::Scalar(0,255,0), 1, 8, 0 );

            int dx = (int)(roi.x + roi.width/2  - rgb_image.cols/2);
            int dy = (int)(roi.y + roi.height/2 - rgb_image.rows/2);
            sprintf(message1,"dx=%04d, dy=%04d",dx, dy);
            
            putText(rgb_image, message1, cv::Point2f(20,30), cv::FONT_HERSHEY_SIMPLEX, 1,  cv::Scalar(0,255,0));
            putText(rgb_image, message2, cv::Point2f(20,60), cv::FONT_HERSHEY_SIMPLEX, 1,  cv::Scalar(0,255,0));
            cv::imshow(RGB_WINDOW_NAME, rgb_image);
        }

        cmd_vel.linear.x  = 0;
        cmd_vel.angular.z = 0;
        
        // 深度追踪（注意，未使用同步帧）
        if (!depth_image.empty())
        {
            end_time = get_wall_time() -  start_time;

            if (tu.getState() == TrackingUtility::STATE_ONGOING)
            {
                float dist_val[5] ;
                //访问矩阵元素 画面对称分布的5个点的距离信息
                dist_val[0] = depth_image.at<float>(roi_result.y + roi_result.height/3 ,      roi_result.x + roi_result.width/3);
                dist_val[1] = depth_image.at<float>(roi_result.y + roi_result.height/3 ,      roi_result.x + roi_result.width/3*2);
                dist_val[2] = depth_image.at<float>(roi_result.y + roi_result.height/3*2 , roi_result.x + roi_result.width/3);
                dist_val[3] = depth_image.at<float>(roi_result.y + roi_result.height/3*2 , roi_result.x + roi_result.width/3*2);
                dist_val[4] = depth_image.at<float>(roi_result.y + roi_result.height/2 ,     roi_result.x + roi_result.width/2);

                int num_depth_points = 5;
                distance = 0;

                for(int i = 0; i < 5; i++)
                {
                    dist_val[i] = dist_val[i] / 1000;
                    //筛选0.4 小于10.0的距离值
                    if(dist_val[i] > 0.4 && dist_val[i] < 10.0)
                    {
                        //叠加符合的距离值
                        distance += dist_val[i];
                    }
                    else //不符合则点数量减1
                    {
                        num_depth_points--;
                    }
                }

                // 取平均距离
                distance /= num_depth_points;
                // ROS_INFO(" distance: %0.4f", distance);

                // 计算中心位置距离
                center_x = (int)(roi_result.x + roi_result.width/2  - rgb_image.cols/2);

                // 深度距离限制
                if (distance > 0 && distance < 10)
                {
                    dcxt = fabs(distance_last - distance); // (m)
                    
                    cyt = - distance * center_x / focus_X;
                    dcyt = fabs(cyt_last - cyt);
                    // ROS_INFO("center_x: %d, cyt: %0.4f, dcyt: %0.4f , distance: %0.4f", center_x, cyt, dcyt, distance);

                    // 储存上一帧数据
                    distance_last = distance ;
                    cyt_last = cyt;

                    // 通过移动速度进行加速反馈
                    target_linear_speed_x = dcxt / end_time;
                    target_linear_speed_y = dcyt / end_time;
                    
                    float target_vel = sqrt(target_linear_speed_x * target_linear_speed_x + target_linear_speed_y * target_linear_speed_y);
                    // ROS_INFO("target_vel: %0.4f, target_linear_speed_x: %0.4f , target_linear_speed_y: %0.4f", target_vel, target_linear_speed_x, target_linear_speed_y);

                    // PID增量控制
                    angular_z = posPID.calculation(cyt, 0) * -1;
                    // angular_z = (target_linear_speed_y + k_angula_speed * cyt) / distance;

                    // 限幅
                    angular_z    = angular_z >= 1.0 ? 1.0 : angular_z;
                    angular_z    = angular_z < -1.0 ? -1.0 : angular_z;

                    linear_x =      target_linear_speed_x + k_linear_speed * ( distance - Min_distance) + cyt * angular_z;
                    linear_x = linear_x > Max_linear_speed ? Max_linear_speed : linear_x;

                    // ROS_INFO("target_vel: %0.4f, linear_x: %0.4f , angular_z: %0.4f", target_vel, linear_x, angular_z);
                    
                    //如果旋转速度过大，则后退
                    if (angular_z > Max_rotation_speed)
                    {
                        angular_z = Max_rotation_speed;
                    }
                    else if (angular_z < -Max_rotation_speed )
                    {
                        angular_z = -Max_rotation_speed;
                    }
                    
                    cmd_vel.linear.x  = linear_x;
                    cmd_vel.angular.z = angular_z;
                }
            }

            start_time = get_wall_time();
        }
        pub_cmd_vel_.publish(cmd_vel);
        
        
        ros::spinOnce();
    }
}

int main(int argc, char** argv)
{
    // 初始化节点
    ros::init(argc, argv, "lingao_kcf_tracking_node");
    ros::NodeHandle nh_("");

    KCF_Tracking tracking(&nh_);
    tracking.loop();

    return 0;
}
