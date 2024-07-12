#include <stdint.h>
#include <stdio.h>

// 1. PID结构体定义
typedef struct 
{
    float setpoint;                 // 目标值
    float error;                    // 当前误差
    float kp;                       // 比例系数
    float ki;                       // 积分系数
    float kd;                       // 微分系数
    float integral_sum;             // 积分和
    float output;                   // 输出值
    float integral_max;             // 积分上限
    float integral_min;             // 积分下限
    float output_max;               // 输出上限
    float output_min;               // 输出下限
    float deadband;                 // 死区
    float prev_output;              // 前一次输出，用于平滑
    float alpha;                    // 平滑系数，用于输出平滑
    float prev_sensor_value;        // 前一次测量值，用于微分先行
    float (*getSensor)(void);       // 函数指针，获取传感器值
} PID;

// 2. 示例传感器获取函数
static inline float getSensor(void) 
{
    return (float)2;  // 模拟传感器值
}

// 3. 初始化PID实例的函数
void initPID(PID* pid, float kp, float ki, float kd, float integral_max, float integral_min, float output_max, float output_min, float deadband, float alpha, float (*getSensor)(void)) 
{
    pid->setpoint = 0.0;
    pid->error = 0.0;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_sum = 0.0;
    pid->output = 0.0;
    pid->integral_max = integral_max;
    pid->integral_min = integral_min;
    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->deadband = deadband;
    pid->prev_output = 0.0;
    pid->alpha = alpha;
    pid->prev_sensor_value = 0.0;
    pid->getSensor = getSensor;
}

// 4. PID控制器函数
void PIDController(PID* pid) {
    // 获取传感器值并确保其有效性
    float sensor_value = pid->getSensor();

    // 计算误差
    pid->error = pid->setpoint - sensor_value;

    // 死区处理
    if (pid->error > -pid->deadband && pid->error < pid->deadband) 
    {
        pid->error = 0.0;
    }

    // 更新积分和并进行积分饱和处理（Anti-windup）
    pid->integral_sum += pid->error;
    if (pid->integral_sum > pid->integral_max) 
    {
        pid->integral_sum = pid->integral_max;
    } 
    else if (pid->integral_sum < pid->integral_min) 
    {
        pid->integral_sum = pid->integral_min;
    }

    // 计算微分项（微分先行）
    float derivative = pid->kd * (sensor_value - pid->prev_sensor_value);

    // 计算PID输出
    pid->output = pid->kp * pid->error + pid->ki * pid->integral_sum - derivative;

    // 输出限幅处理
    if (pid->output > pid->output_max) 
    {
        pid->output = pid->output_max;
    } 
    else if (pid->output < pid->output_min) 
    {
        pid->output = pid->output_min;
    }

    // 输出平滑处理
    pid->output = pid->alpha * pid->output + (1 - pid->alpha) * pid->prev_output;

    // 更新前一次输出和前一次测量值
    pid->prev_output = pid->output;
    pid->prev_sensor_value = sensor_value;
}

// 5. 主函数示例
int main() {
    // 初始化PID实例
    PID pid1;
    initPID(&pid1, 1.0, 1.0, 1.0, 100.0, -100.0, 50.0, -50.0, 0.5, 0.9, getSensor);

    // 设置目标值
    pid1.setpoint = 10.0;

    // 调用PID控制器
    PIDController(&pid1);

    // 把pid1.output作用给相应执行器
    
    return 0;
}
