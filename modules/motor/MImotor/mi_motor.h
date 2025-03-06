#ifndef MI_MOTOR_H
#define MI_MOTOR_H

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define MI_MOTOR_CNT 12


/* Private defines -----------------------------------------------------------*/
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f

#define IQ_REF_MIN -27.0f
#define IQ_REF_MAX 27.0f
#define SPD_REF_MIN -30.0f
#define SPD_REF_MAX 30.0f
#define LIMIT_TORQUE_MIN 0.0f
#define LIMIT_TORQUE_MAX 12.0f
#define CUR_FILT_GAIN_MIN 0.0f
#define CUR_FILT_GAIN_MAX 1.0f
#define LIMIT_SPD_MIN 0.0f
#define LIMIT_SPD_MAX 30.0f
#define LIMIT_CUR_MIN 0.0f
#define LIMIT_CUR_MAX 27.0f

typedef enum
{
    OK                 = 0,//无故障
    BAT_LOW_ERR        = 1,//欠压故障
    OVER_CURRENT_ERR   = 2,//过流
    OVER_TEMP_ERR      = 3,//过温
    MAGNETIC_ERR       = 4,//磁编码故障
    HALL_ERR_ERR       = 5,//HALL编码故障
    NO_CALIBRATION_ERR = 6//未标定
}motor_state_e;//电机状态（故障信息）

typedef enum
{
    CONTROL_MODE  = 0, //运控模式
    LOCATION_MODE = 1, //位置模式
    SPEED_MODE    = 2, //速度模式
    CURRENT_MODE  = 3  //电流模式
} motor_run_mode_e;//电机运行模式

typedef enum
{
    IQ_REF        = 0X7006,//电流模式Iq指令
    SPD_REF       = 0X700A,//转速模式转速指令
    LIMIT_TORQUE  = 0X700B,//转矩限制
    CUR_KP        = 0X7010,//电流的 Kp 
    CUR_KI        = 0X7011,//电流的 Ki 
    CUR_FILT_GAIN = 0X7014,//电流滤波系数filt_gain
    LOC_REF       = 0X7016,//位置模式角度指令
    LIMIT_SPD     = 0X7017,//位置模式速度设置
    LIMIT_CUR     = 0X7018 //速度位置模式电流设置
} motor_index_e;//电机功能码

typedef enum
{
    RESET_MODE = 0,//Reset模式[复位]
    CALI_MODE  = 1,//Cali 模式[标定]
    RUN_MODE   = 2 //Motor模式[运行]
} motor_mode_state_e;//电机模式状态



typedef struct
{
    CANInstance *motor_can_instace;
    motor_state_e motor_state;
    motor_mode_state_e  motor_mode_state;
    uint8_t motor_id;
    RxCAN_info_type_2_s RxCAN_info;
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;
}MIMotorInstance;

/**********************Functions**************************/

MIMotorInstance *MIMotorInit(Motor_Init_Config_s *config);


void MI_motor_GetID(MIMotorInstance* motor);
void DecodeMiMotor(CANInstance *_instance);
void MIMotorEnable(MIMotorInstance *motor);
void MIMotorInstancetop(MIMotorInstance *motor);
void MIMotorInstanceetMechPositionToZero(MIMotorInstance *motor);
void MI_motor_ChangeID(MIMotorInstance* motor,uint8_t Now_ID,uint8_t Target_ID);
void MI_motor_ReadParam(MIMotorInstance* motor, uint16_t index);

void MIMotorModeSwitch(MIMotorInstance* motor, uint8_t run_mode);
void MI_motor_TorqueControl(MIMotorInstance* motor, float torque);
void MI_motor_LocationControl(MIMotorInstance* motor, float location, float kp, float kd);
void MIMotorInstancepeedControl(MIMotorInstance* motor, float speed, float kd);
void MiMotorControl();
void MIMotorSetPid(MIMotorInstance* motor, float location_kp,float limit_speed,float speed_kp,float speed_ki);
void MiMotorSetRef(MIMotorInstance* motor,float location_ref);

#endif