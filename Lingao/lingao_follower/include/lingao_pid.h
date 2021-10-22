/*
 * Copyright (C) 2021, LingAo Robotics, INC.
 * @Version: V1.0
 * @Author: 神炽焰
 * @Date: 2021-09-06 20:29:35
 * @LastEditTime: 2021-09-06 20:29:53
 * @LastEditors: owen
 * @Description: 
 * @FilePath: /lingao_ws/src/lingaoRobot/lingao_visual/lingao_line_follower/include/lingao_pid.h
 */
#ifndef __PID_H
#define __PID_H

/**************************************************************************
文件版本：神炽焰 V1.0
文件功能：
1、增量式PID参数初始化
2、位置式PID参数初始化
3、增量式PID计算函数
4、位置式PID计算函数
**************************************************************************/
#include "lingao_pid.h"

class Lingao_IncPID {
  private:
    float Proportion; //比例常数 Proportional Const
    float Integral;   //积分常数 Integral Const
    float Derivative; //微分常数 Derivative Const

    float Error;     // E(k)
    float PrevError; // E(k-1)
    float LastError; // E(k-2)
  public:
    Lingao_IncPID(float P, float I, float D); //增量式PID参数初始化
    void reset();
    float calculation(float NextPoint, float SetPoint); //增量式PID计算函数
};

void Lingao_IncPID::reset()
{
    Error     = 0; // E(k)
    PrevError = 0; // E(k-1)
    LastError = 0; // E(k-2)
}

/**************************************************************************
函数功能：增量式PID参数初始化
入口参数：比例参数，积分参数，微分参数
返回  值：None
**************************************************************************/
Lingao_IncPID::Lingao_IncPID(float P, float I, float D)
{
    Proportion = P; //比例常数 Proportional Const
    Integral   = I; //积分常数Integral Const
    Derivative = D; //微分常数 Derivative Const

    Error     = 0; // E(k)
    PrevError = 0; // E(k-1)
    LastError = 0; // E(k-2)
}

/**************************************************************************
函数功能：增量式PID计算函数
入口参数：测量值，目标值
返回  值：电机PWM
根据增量式离散PID公式
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差  以此类推
pwm代表增量输出
**************************************************************************/
float Lingao_IncPID::calculation(float NextPoint, float SetPoint)
{
    Error = SetPoint - NextPoint;

    float Value = Proportion * (Error - PrevError) + Integral * Error + Derivative * (Error - 2 * PrevError + LastError); //增量计算

    LastError = PrevError; //下一次迭代
    PrevError = Error;
    return Value;
}

class Lingao_PosPID {
  private:
    float Proportion; //比例常数 Proportional Const
    float Integral;   //积分常数 Integral Const
    float Derivative; //微分常数 Derivative Const

    float Error;     // E(k)
    float PrevError; // E(k-1)
    float LastError; // E(k-2)

    float IntegralError;     //累积误差值
    float IntegralError_Min; //累积误差值限幅最小值
    float IntegralError_Max; //累积误差值限幅最大值
  public:
    Lingao_PosPID(float P, float I, float D, float IntegralError_Min_, float IntegralError_Max_); //位置式PID参数初始化
    void reset();
    float calculation(float NextPoint, float SetPoint); //位置式PID计算函数
};

void Lingao_PosPID::reset()
{
    Error         = 0; // E(k)
    PrevError     = 0; // E(k-1)
    IntegralError = 0; //累积误差值
}

/**************************************************************************
函数功能：位置式PID参数初始化
入口参数：比例参数，积分参数，微分参数
返回  值：None
**************************************************************************/
Lingao_PosPID::Lingao_PosPID(float P, float I, float D, float IntegralError_Min_, float IntegralError_Max_)
{
    Proportion = P; //比例常数 Proportional Const
    Integral   = I; //积分常数Integral Const
    Derivative = D; //微分常数 Derivative Const

    Error     = 0; // E(k)
    PrevError = 0; // E(k-1)

    IntegralError     = 0;                  //累积误差值
    IntegralError_Min = IntegralError_Min_; //累积误差值限幅最小值
    IntegralError_Max = IntegralError_Max_; //累积误差值限幅最大值
}

/**************************************************************************
函数功能：位置式PID计算函数
入口参数：测量值，目标值
返回  值：电机PWM
根据位置式离散PID公式
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
**************************************************************************/
float Lingao_PosPID::calculation(float NextPoint, float SetPoint)
{
    Error = SetPoint - NextPoint;

    IntegralError += Error;
    if (IntegralError > IntegralError_Max)
        IntegralError = IntegralError_Max; //积分限幅，防止累积值一直增大
    else if (IntegralError < IntegralError_Min)
        IntegralError = IntegralError_Min;

    float PositionPid = Proportion * Error + Integral * IntegralError + Derivative * (Error - PrevError);

    PrevError = Error;
    return PositionPid;
}

#endif
