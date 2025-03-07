#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "mi_motor.h"
#include "bmi088.h"

static attitude_t *gimbal_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor;
static MIMotorInstance *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static uint8_t motor_init=0;
void GimbalInit()
{
    gimbal_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 30, // 8
                .Ki = 20,
                .Kd = 1.2,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,

                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 50,  // 50
                .Ki = 200, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config={
            .angle_PID={
                .Kp=10,
                .Kd=0.35,
            },
            .speed_PID={
                .Kp=1,
                .Ki=0.01,
            },
        },
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = MIMotorInit(&pitch_config);
    //MIMotorModeSwitch(pitch_motor,1);
    //MIMotorSetPid(pitch_motor,pitch_motor->motor_controller.angle_PID.Kp,4,pitch_motor->motor_controller.speed_PID.Kp,pitch_motor->motor_controller.speed_PID.Ki);
    MIMotorInstanceetMechPositionToZero(pitch_motor);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

static void GimbalStateSet()
{
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        MIMotorInstancetop(pitch_motor);
        DJIMotorStop(yaw_motor);
        motor_init=0;
        break;
    case GIMBAL_GYRO_MODE: 
        DJIMotorEnable(yaw_motor);
        DJIMotorSetRef(yaw_motor,gimbal_cmd_recv.yaw);
        MI_motor_LocationControl(pitch_motor,gimbal_cmd_recv.pitch,pitch_motor->motor_controller.angle_PID.Kp,pitch_motor->motor_controller.angle_PID.Kd);
        if(motor_init==0)
        {
            //MIMotorModeSwitch(pitch_motor,1);
            MIMotorEnable(pitch_motor);
            gimbal_feedback_data.init_location=gimbal_IMU_data->Pitch;

            //MIMotorSetPid(pitch_motor,pitch_motor->motor_controller.angle_PID.Kp,4,pitch_motor->motor_controller.speed_PID.Kp,pitch_motor->motor_controller.speed_PID.Ki);
            motor_init=1;
        }
        /*
        else
        MiMotorSetRef(pitch_motor,gimbal_cmd_recv.pitch);
        */
        break;
    default:
        break;
    }
}



static void SendGimbalData()
{
    gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
}

/* 机器人云台控制核心任务 */
void GimbalTask()
{
    // 获取云台控制数据
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    //云台启停
    GimbalStateSet();
    // 设置反馈数据,主要是imu和yaw的ecd
    SendGimbalData();
    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}