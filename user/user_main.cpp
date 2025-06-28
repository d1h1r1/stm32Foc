#include "user_main.h"
#include "AS5600_I2C.h"
#include "BLDCDriver3PWM.h"
#include "BLDCMotor.h"
#include "InlineCurrentSense.h"

// 外部变量
extern struct AS5600_I2CConfig_s AS5600_I2C_Config;
extern I2C_HandleTypeDef hi2c1;

AS5600_I2C AS5600_1(AS5600_I2C_Config); // 创建AS5600_I2C对象
BLDCDriver3PWM motorDriver(GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2); // PA0,PA1,PA2
BLDCMotor motor(11); // 创建BLDCMotor对象,电机是7对极
InlineCurrentSense currentSense(0.001f,50.0f,ADC_CHANNEL_3,ADC_CHANNEL_4,NOT_SET); // 创建电流传感器对象

float targetAngle = 3.0f; // 目标角度
float curAngle = 0.0f; // 当前角度

/**
 * @brief C++环境入口函数
 * 
 */
void main_Cpp(void)
{
    AS5600_1.init(&hi2c1); // 初始化AS5600
    motorDriver.voltage_power_supply = DEF_POWER_SUPPLY; // 设置电压
    motorDriver.init();   // 初始化电机驱动

    currentSense.skip_align = true; // 跳过检测电机三相接线
    currentSense.init();   // 初始化电流传感器
    currentSense.linkDriver(&motorDriver); // 电流传感器连接驱动器
                            
    motor.linkSensor(&AS5600_1); // 连接编码器
    motor.linkDriver(&motorDriver); // 连接驱动器
    motor.linkCurrentSense(&currentSense); // 连接电流传感器
    motor.voltage_sensor_align = 6; // 校准偏移offset时，所用到的电压值（相当于占空比4V / 12V = 1/3）
    motor.controller = MotionControlType::angle; // 设置控制器模式(位置闭环模式)

    motor.PID_velocity.P = 0.50f; // 设置速度P
    motor.PID_velocity.I = 0.0f; // 设置速度I
    motor.PID_velocity.D = 0; // 设置速度D
    motor.PID_velocity.output_ramp = 0; // 0：不设置斜坡
    motor.LPF_velocity.Tf = 0.01f; // 设置速度低通滤波器

    motor.P_angle.P = 10.0f; // 位置环P
    motor.P_angle.I = 0.0f; // 位置环I
    motor.P_angle.D = 0.0f;  // 位置环D
    motor.P_angle.output_ramp = 0; // 不设置
    
    motor.PID_current_q.P = 2.0f;
    motor.PID_current_q.I = 0.0f;
    motor.PID_current_q.D = 0;
    motor.PID_current_q.output_ramp = 0; // 不设置
    motor.LPF_current_q.Tf = 0.01f;      // 低通滤波器
    
    motor.PID_current_d.P = 2.0;
    motor.PID_current_d.I = 0.0f;
    motor.PID_current_d.D = 0;
    motor.PID_current_d.output_ramp = 0; // 0：不设置斜坡
    motor.LPF_current_d.Tf = 0.01f;
    
    motor.current_limit = DEF_CURRENT_LIMIT; // 电流限制
    motor.voltage_limit = DEF_VOLTAGE_LIMIT; // 电压限制
    motor.velocity_limit = DEF_VELOCITY_LIMIT; // 位置闭环模式时，变成位置环PID的limit
    motor.torque_controller = TorqueControlType::dc_current; // Iq闭环，Id = 0
    
    motor.init(); // 初始化电机

    motor.PID_current_q.limit = DEF_VOLTAGE_LIMIT;
    motor.PID_current_d.limit = DEF_VOLTAGE_LIMIT;

    motor.foc_modulation = FOCModulationType::SpaceVectorPWM; // 正弦波改为马鞍波
    motor.sensor_direction = Direction::CCW; // 之前校准传感器的时候，知道传感器的方向是CCW（翻开校准传感器的章节就知道）
    motor.initFOC(); // 初始化FOC

    // (motor.zero_electric_angle); // 打印电机零电角度
    // (AS5600_1.getMechanicalAngle()); // 打印传感器角度

    HAL_Delay(1000); // 延时1s
    HAL_TIM_Base_Start_IT(&htim4); // 启动TIM4定时器

    while(1) {
        HAL_GPIO_TogglePin(run_led_GPIO_Port,run_led_Pin); // 心跳灯跑起来
        curAngle = motor.shaft_angle; // 获取当前位置
        // SEGGER_Printf_Float(curAngle); // 打印当前位置
        // SEGGER_Printf_Float(targetAngle); // 打印目标位置
        delayMicroseconds(100000U); // 延时100ms
    }
}
/**
 * @brief 定时器中断回调函数
 * 
 * @param htim 定时器句柄
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
    } else if(htim->Instance == TIM4) {    
        motor.loopFOC(); // 执行FOC
        motor.move(targetAngle); // 控制目标角度     

        // // 将占空比放大10倍，便于观察
        // JS_Message.a = motor.driver->dc_a * 10; // A相占空比
        // JS_Message.b = motor.driver->dc_b * 10; // B相占空比
        // JS_Message.c = motor.driver->dc_c * 10; // C相占空比
    }
}
