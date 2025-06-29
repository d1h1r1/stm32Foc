#include "user_main.h"
#include "AS5600_I2C.h"
#include "BLDCDriver3PWM.h"
#include "BLDCMotor.h"
#include "InlineCurrentSense.h"

// �ⲿ����
extern struct AS5600_I2CConfig_s AS5600_I2C_Config;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

AS5600_I2C AS5600_1(AS5600_I2C_Config); // ����AS5600_I2C����
BLDCDriver3PWM motorDriver(GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2); // PA0,PA1,PA2
BLDCMotor motor(11); // ����BLDCMotor����,�����7�Լ�
InlineCurrentSense currentSense(0.001f,50.0f,ADC_CHANNEL_3,ADC_CHANNEL_4,NOT_SET); // ������������������

float targetAngle = 3.0f; // Ŀ��Ƕ�
float curAngle = 0.0f; // ��ǰ�Ƕ�

/**
 * @brief C++������ں���
 * 
 */
void main_Cpp(void)
{
    AS5600_1.init(&hi2c1, &hi2c2); // ��ʼ��AS5600
    motorDriver.voltage_power_supply = DEF_POWER_SUPPLY; // ���õ�ѹ
    motorDriver.init();   // ��ʼ���������

    currentSense.skip_align = true; // ����������������
    currentSense.init();   // ��ʼ������������
    currentSense.linkDriver(&motorDriver); // ��������������������
                            
    motor.linkSensor(&AS5600_1); // ���ӱ�����
    motor.linkDriver(&motorDriver); // ����������
    motor.linkCurrentSense(&currentSense); // ���ӵ���������
    motor.voltage_sensor_align = 6; // У׼ƫ��offsetʱ�����õ��ĵ�ѹֵ���൱��ռ�ձ�4V / 12V = 1/3��
    motor.controller = MotionControlType::angle; // ���ÿ�����ģʽ(λ�ñջ�ģʽ)

    motor.PID_velocity.P = 0.50f; // �����ٶ�P
    motor.PID_velocity.I = 0.0f; // �����ٶ�I
    motor.PID_velocity.D = 0; // �����ٶ�D
    motor.PID_velocity.output_ramp = 0; // 0��������б��
    motor.LPF_velocity.Tf = 0.01f; // �����ٶȵ�ͨ�˲���

    motor.P_angle.P = 10.0f; // λ�û�P
    motor.P_angle.I = 0.0f; // λ�û�I
    motor.P_angle.D = 0.0f;  // λ�û�D
    motor.P_angle.output_ramp = 0; // ������
    
    motor.PID_current_q.P = 2.0f;
    motor.PID_current_q.I = 0.0f;
    motor.PID_current_q.D = 0;
    motor.PID_current_q.output_ramp = 0; // ������
    motor.LPF_current_q.Tf = 0.01f;      // ��ͨ�˲���
    
    motor.PID_current_d.P = 2.0;
    motor.PID_current_d.I = 0.0f;
    motor.PID_current_d.D = 0;
    motor.PID_current_d.output_ramp = 0; // 0��������б��
    motor.LPF_current_d.Tf = 0.01f;
    
    motor.current_limit = DEF_CURRENT_LIMIT; // ��������
    motor.voltage_limit = DEF_VOLTAGE_LIMIT; // ��ѹ����
    motor.velocity_limit = DEF_VELOCITY_LIMIT; // λ�ñջ�ģʽʱ�����λ�û�PID��limit
    motor.torque_controller = TorqueControlType::dc_current; // Iq�ջ���Id = 0
    
    motor.init(); // ��ʼ�����

    motor.PID_current_q.limit = DEF_VOLTAGE_LIMIT;
    motor.PID_current_d.limit = DEF_VOLTAGE_LIMIT;

    motor.foc_modulation = FOCModulationType::SpaceVectorPWM; // ���Ҳ���Ϊ������
    motor.sensor_direction = Direction::CCW; // ֮ǰУ׼��������ʱ��֪���������ķ�����CCW������У׼���������½ھ�֪����
    motor.initFOC(); // ��ʼ��FOC

    // (motor.zero_electric_angle); // ��ӡ������Ƕ�
    // (AS5600_1.getMechanicalAngle()); // ��ӡ�������Ƕ�

    HAL_Delay(1000); // ��ʱ1s
    HAL_TIM_Base_Start_IT(&htim4); // ����TIM4��ʱ��

    while(1) {
        HAL_GPIO_TogglePin(run_led_GPIO_Port,run_led_Pin); // ������������
        curAngle = motor.shaft_angle; // ��ȡ��ǰλ��
        // SEGGER_Printf_Float(curAngle); // ��ӡ��ǰλ��
        // SEGGER_Printf_Float(targetAngle); // ��ӡĿ��λ��
        delayMicroseconds(100000U); // ��ʱ100ms
    }
}
/**
 * @brief ��ʱ���жϻص�����
 * 
 * @param htim ��ʱ�����
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
    } else if(htim->Instance == TIM4) {    
        motor.loopFOC(); // ִ��FOC
        motor.move(targetAngle); // ����Ŀ��Ƕ�     

        // // ��ռ�ձȷŴ�10�������ڹ۲�
        // JS_Message.a = motor.driver->dc_a * 10; // A��ռ�ձ�
        // JS_Message.b = motor.driver->dc_b * 10; // B��ռ�ձ�
        // JS_Message.c = motor.driver->dc_c * 10; // C��ռ�ձ�
    }
}
