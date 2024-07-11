#include <stdint.h>
#include <stdio.h>

//1.PID结构体定义
typedef struct {
    float setpoint;                 // 目标值
    float error;                    // 当前误差
    float error_last;               // 上一次误差
    float kp;                       // 比例系数
    float ki;                       // 积分系数
    float kd;                       // 微分系数
    float integral_sum;             // 积分和
    float output;                   // 输出值
    float integral_max;             // 积分上限
    float integral_min;             // 积分下限
    int32_t (*getSensor)(void);     // 函数指针，获取传感器值
} PID;

//2.示例传感器获取函数
inline int32_t get(void) {
    return 2;  // 模拟传感器值
}

//3.初始化PID实例的函数
void initPID(PID* pid, float kp, float ki, float kd, float integral_max, float integral_min, int32_t (*getSensor)(void)) {
    pid->setpoint = 0.0;
    pid->error = 0.0;
    pid->error_last = 0.0;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_sum = 0.0;
    pid->output = 0.0;
    pid->integral_max = integral_max;
    pid->integral_min = integral_min;
    pid->getSensor = getSensor;
}
	
//4.PID控制器函数
void PIDController(PID* pid) {
    // 获取传感器值并确保其有效性
    int32_t sensor_value = pid->getSensor();

    // 计算误差
    pid->error = pid->setpoint - sensor_value;

    // 更新积分和并进行积分饱和处理
    pid->integral_sum += pid->error;
    if (pid->integral_sum > pid->integral_max) 
    pid->integral_sum = pid->integral_max;
	else if (pid->integral_sum < pid->integral_min) 
    pid->integral_sum = pid->integral_min;

    // 计算PID输出
    pid->output = pid->kp * pid->error + pid->ki * pid->integral_sum + pid->kd * (pid->error - pid->error_last);

    // 更新上一次误差
    pid->error_last = pid->error;
}

//5.主函数示例
int main() {
    // 初始化PID实例
    PID pid1;
    initPID(&pid1, 1.0, 1.0, 1.0, 100.0, -100.0, get);

    // 设置目标值
    pid1.setpoint = 10.0;

    // 调用PID控制器
    PIDController(&pid1);

	//把pid->output作用给相应执行器

    return 0;
}