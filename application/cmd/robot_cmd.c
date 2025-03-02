// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "buzzer.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Minipc_Recv_s *minipc_recv_data; // 视觉接收数据指针,初始化时返回
static Minipc_Send_s minipc_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态
static  BuzzzerInstance *aim_success_buzzer;
static DataLebel_t DataLebel;

uint8_t gimbal_location_init=0;
void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    minipc_recv_data = minipcInit(&huart1); // 视觉通信串口

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    gimbal_cmd_send.pitch = 0;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入

    Buzzer_config_s aim_success_buzzer_config= {
        .alarm_level=ALARM_LEVEL_ABOVE_MEDIUM,
        .octave=OCTAVE_2,
    };
    aim_success_buzzer= BuzzerRegister(&aim_success_buzzer_config);
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

static void BasicSet()
{
    // 云台软件限位
    if(gimbal_cmd_send.pitch<PITCH_MIN_ANGLE)
    gimbal_cmd_send.pitch=PITCH_MIN_ANGLE;
    else if (gimbal_cmd_send.pitch>PITCH_MAX_ANGLE)
    gimbal_cmd_send.pitch=PITCH_MAX_ANGLE;
    else
    gimbal_cmd_send.pitch=gimbal_cmd_send.pitch;

    //发射基本模式设定
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_ON;
    shoot_cmd_send.shoot_rate=8;
    shoot_cmd_send.bullet_speed=SMALL_AMU_30;
    
    if(rc_data->rc.dial>200)
    shoot_cmd_send.load_mode=LOAD_BURSTFIRE;
    else if (rc_data->rc.dial<-200)
    {
        shoot_cmd_send.load_mode=LOAD_REVERSE;
    }
    else
    shoot_cmd_send.load_mode=LOAD_STOP;
}


static void GimbalRC()
{
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    gimbal_cmd_send.yaw -= 0.0005f * (float)rc_data[TEMP].rc.rocker_right_x;//0
    gimbal_cmd_send.pitch -= 0.0001f * (float)rc_data[TEMP].rc.rocker_right_y;
    gimbal_cmd_send.real_pitch= (gimbal_cmd_send.pitch*57.29578-gimbal_fetch_data.init_location);
}

static void GimbalAC()
{
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    if(minipc_recv_data->Vision.yaw>1)
    {
        gimbal_cmd_send.yaw-=0.007f*minipc_recv_data->Vision.yaw;   //往右获得的yaw是减
    }
    if(minipc_recv_data->Vision.pitch>1)
    {
        gimbal_cmd_send.pitch -= 0.009f*minipc_recv_data->Vision.pitch;
    }
}

static void ChassisRC()
{
    chassis_cmd_send.vx = 10.0f * (float)rc_data[TEMP].rc.rocker_left_y; // _水平方向
    chassis_cmd_send.vy =-10.0f * (float)rc_data[TEMP].rc.rocker_left_x; // 竖直方向

    if (switch_is_down(rc_data[TEMP].rc.switch_left))
    {
        chassis_cmd_send.chassis_mode=CHASSIS_FOLLOW_GIMBAL_YAW;
    }
    else
        chassis_cmd_send.chassis_mode=CHASSIS_ROTATE;
}

/**
 * @brief 判断视觉有没有发信息，但因为现在发信息发的太慢了，头一直想走，所以给视觉3s的机会
 *
 */
static void VisionJudge()
{
    if(minipc_recv_data->Vision.deep!=0)//代表收到信息
    {
        DataLebel.vision_flag=1;
        AlarmSetStatus(aim_success_buzzer, ALARM_ON);
        if(abs(minipc_recv_data->Vision.yaw)>1&&aim_success_buzzer->loudness<0.5)
        {
            aim_success_buzzer->loudness=0.5*(1/abs(minipc_recv_data->Vision.yaw));
        }
        else if(abs(minipc_recv_data->Vision.yaw)<1 && abs(minipc_recv_data->Vision.pitch)<1)
        {
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        }
    }
    else if(minipc_recv_data->Vision.deep==0 && DataLebel.vision_flag==1)       
    {
        shoot_cmd_send.load_mode = LOAD_STOP;
        DataLebel.t_shoot++;
        if(minipc_recv_data->Vision.deep!=0)
        {
            DataLebel.t_shoot=0;
        }
        if(DataLebel.t_shoot>=600)
        {
            DataLebel.t_shoot=0;
            DataLebel.vision_flag=0;
            AlarmSetStatus(aim_success_buzzer, ALARM_OFF);
        }
    }
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    VisionJudge();
    if(DataLebel.vision_flag==1)    
    {
        GimbalAC();
    }
    else
    {
        GimbalRC();
    }
    ChassisRC();
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].w * 300 - rc_data[TEMP].key[KEY_PRESS].s * 300; // 系数待测
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].s * 300 - rc_data[TEMP].key[KEY_PRESS].d * 300;

    gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
    gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;

    switch (rc_data[TEMP].mouse.press_l) // 鼠标左键射击
    {
    case 0:
        shoot_cmd_send.load_mode = LOAD_STOP;
        break;
    default:
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
        if(chassis_fetch_data.over_heat_flag==1)
        {
            shoot_cmd_send.load_mode = LOAD_STOP;
            break;
        }
        break;
    }

    switch (rc_data[TEMP].mouse.press_l) // 鼠标右键（暴走模式）
    {
    case 1:
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    }

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
    {
    case 0:
        chassis_cmd_send.chassis_speed_buff = 40;
        break;
    case 1:
        chassis_cmd_send.chassis_speed_buff = 60;
        break;
    case 2:
        chassis_cmd_send.chassis_speed_buff = 80;
        break;
    default:
        chassis_cmd_send.chassis_speed_buff = 100;
        break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    {
    case 1:
        break;
    default:
        break;
    }
}

/**
 * @brief 控制量及模式设置
 *
 */
static void ControlDataDeal()
{
    if (switch_is_mid(rc_data[TEMP].rc.switch_right)) 
    {
        BasicSet();
        RemoteControlSet();
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right)) 
    {
        MouseKeySet();   
    }
        // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 还需添加重要应用和模块离线的判断
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{

}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    // 处理模块离线和遥控器急停等紧急情况
    EmergencyHandler(); 
    ControlDataDeal();
    // 设置视觉发送数据,还需增加加速度和角速度数据
    VisionSetFlag();
    VisionSetAltitude(gimbal_fetch_data.gimbal_imu_data.Yaw,gimbal_fetch_data.gimbal_imu_data.Pitch,gimbal_fetch_data.gimbal_imu_data.Roll);
    // 推送消息,双板通信,视觉通信等
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    SendMinipcData(&minipc_send_data);
}
