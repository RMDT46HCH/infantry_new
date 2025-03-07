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
#include "referee_UI.h"
#include "referee_task.h"

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

static uint8_t gimbal_location_init=0;
static uint8_t power_flag;

static referee_info_t* referee_data; // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI
static float cnt1,cnt2; 
static float chassis_rotate_buff;
static float chassis_speed_buff;

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    minipc_recv_data = minipcInit(&huart1); // 视觉通信串口
    referee_data= UITaskInit(&huart6,&ui_data);

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    gimbal_cmd_send.pitch = 0;

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

static void GimbalPitchLimit()
{
    gimbal_cmd_send.gimbal_mode=GIMBAL_GYRO_MODE;
    // 云台软件限位
    if(gimbal_cmd_send.pitch<PITCH_MIN_ANGLE)
    gimbal_cmd_send.pitch=PITCH_MIN_ANGLE;
    else if (gimbal_cmd_send.pitch>PITCH_MAX_ANGLE)
    gimbal_cmd_send.pitch=PITCH_MAX_ANGLE;
    else
    gimbal_cmd_send.pitch=gimbal_cmd_send.pitch;
}

/**
 * @brief 判断视觉有没有发信息
 *
 */
static void VisionJudge()
{
    //cnt1用于检测小电脑的离线，取值为[-1,1]
    //在-0.1到1且小电脑未离线时，读取深度
    cnt1=sin(DWT_GetTimeline_s());
    if(cnt1>-0.1&&cnt1<1&&DataLebel.cmd_error_flag==0)
    {
        gimbal_cmd_send.last_deep= minipc_recv_data->Vision.deep;
    }
    //有深度代表有视觉信息
    if(minipc_recv_data->Vision.deep!=0&&DataLebel.cmd_error_flag==0)//代表收到信息
    {
        DataLebel.aim_flag=1;
        //检测到装甲板，开启蜂鸣器
        AlarmSetStatus(aim_success_buzzer, ALARM_ON);
        //与装甲板中心的距离越近，蜂鸣器越响
        if(abs(minipc_recv_data->Vision.yaw)>1&&aim_success_buzzer->loudness<0.5)
        {
            aim_success_buzzer->loudness=0.5*(1/abs(minipc_recv_data->Vision.yaw));
        }
        else if(abs(minipc_recv_data->Vision.yaw)<1 && abs(minipc_recv_data->Vision.pitch)<1)
        {
            //离装甲板距离较近时，开火
            aim_success_buzzer->loudness=0.5;
            if(DataLebel.reverse_flag==1)
            {
                DataLebel.fire_flag=0;
            }
            else
            {
                DataLebel.fire_flag=1;
            }
        }
        //在cnt1<-0.2时，此时不读取深度，但如果之前读取到的深度与实际深度一致，证明小电脑离线，停止自瞄
        if(minipc_recv_data->Vision.deep-gimbal_cmd_send.last_deep==0&&cnt1<-0.2)
        {
            DataLebel.cmd_error_flag=1;
            DataLebel.fire_flag=0;
            DataLebel.aim_flag=0;
            AlarmSetStatus(aim_success_buzzer, ALARM_OFF);
        }
    }
     //检测不到装甲板，关蜂鸣器，关火
    else if(minipc_recv_data->Vision.deep==0 && DataLebel.aim_flag==1)       
    {
        DataLebel.fire_flag=0;
        DataLebel.aim_flag=0;
        AlarmSetStatus(aim_success_buzzer, ALARM_OFF);    
    }
}

static void BasicSet()
{
    GimbalPitchLimit();
    VisionJudge();
    //发射基本模式设定
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.friction_mode = FRICTION_ON;
    shoot_cmd_send.shoot_rate=8;
    chassis_cmd_send.power_limit=referee_data->GameRobotState.chassis_power_limit;
}


static void GimbalRC()
{
    gimbal_cmd_send.yaw -= 0.0005f * (float)rc_data[TEMP].rc.rocker_right_x;//0
    gimbal_cmd_send.pitch -= 0.0001f * (float)rc_data[TEMP].rc.rocker_right_y;
    gimbal_cmd_send.real_pitch= ((gimbal_fetch_data.gimbal_imu_data.Pitch)-gimbal_fetch_data.init_location)/57.39;
}

static void GimbalAC()
{
    gimbal_cmd_send.yaw-=0.007f*minipc_recv_data->Vision.yaw;   //往右获得的yaw是减
    gimbal_cmd_send.pitch -= 0.009f*minipc_recv_data->Vision.pitch;
}


static void ChassisRC()
{
    chassis_cmd_send.vx = 30.0f * (float)rc_data[TEMP].rc.rocker_left_y; // _水平方向
    chassis_cmd_send.vy =-30.0f * (float)rc_data[TEMP].rc.rocker_left_x; // 竖直方向

    if (switch_is_down(rc_data[TEMP].rc.switch_left))
    {
        chassis_cmd_send.chassis_mode=CHASSIS_FOLLOW_GIMBAL_YAW;
    }
    else
        chassis_cmd_send.chassis_mode=CHASSIS_ROTATE;
}

static void ChassisRotateSet()
{
    // 根据控制模式设定旋转速度
    switch (chassis_cmd_send.chassis_mode)
    {
        //底盘跟随就不调了，懒
        case CHASSIS_FOLLOW_GIMBAL_YAW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
            chassis_cmd_send.wz =-2.0*abs(chassis_cmd_send.offset_angle)*chassis_cmd_send.offset_angle;
        break;
        case CHASSIS_ROTATE: // 变速小陀螺
            chassis_cmd_send.wz = 4000*chassis_cmd_send.chassis_rotate_buff;
        break;
        default:
        break;
    }
}



static void AutoAimSet()
{
    if(DataLebel.aim_flag==1)
    {
        GimbalAC();
        if(DataLebel.fire_flag==1)
        {
            shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
        }
    }
}

static void ShootRC()
{
    if(rc_data->rc.dial>200)
    {
        shoot_cmd_send.loader_mode=LOAD_BURSTFIRE;
    }
    else if (rc_data->rc.dial<-200)
    {
        shoot_cmd_send.loader_mode=LOAD_REVERSE;
        DataLebel.reverse_flag=1;
    }
    else
    {
        shoot_cmd_send.loader_mode=LOAD_STOP;
        DataLebel.reverse_flag=0;
    }
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    ChassisRC();
    if(switch_is_up(rc_data[TEMP].rc.switch_left)) 
    {
        AutoAimSet();
        if(DataLebel.aim_flag!=1)
        {
            gimbal_cmd_send.autoaim_mode=AUTO_ON;
            ShootRC();
            GimbalRC();
        }
        else
        {
            gimbal_cmd_send.autoaim_mode=FIND_Enermy;
        }
    }
    else
    {
        gimbal_cmd_send.autoaim_mode=AUTO_OFF;
        GimbalRC();
        ShootRC();
    }
}
static void NoneAutoMouseControl()
{
    gimbal_cmd_send.yaw -= (float)rc_data[TEMP].mouse.x / 660 *3 ; 
    gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660/57 ;
    if(rc_data[TEMP].mouse.press_l==1)
    {
        if(DataLebel.reverse_flag==1)
        {
            shoot_cmd_send.loader_mode = LOAD_REVERSE;
        }
        else
        {
            shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
        }
    }
    else
    {
        shoot_cmd_send.loader_mode = LOAD_STOP;
    }            
}
static void MouseControl()
{
    DataLebel.aim_flag=0;
    if(rc_data[TEMP].mouse.press_r==1)
    {
        if(DataLebel.aim_flag!=1)
        {
            gimbal_cmd_send.autoaim_mode=AUTO_ON;
        }
        else
        {
            gimbal_cmd_send.autoaim_mode=FIND_Enermy;
        }
    }
    else
    {
        gimbal_cmd_send.autoaim_mode=AUTO_OFF;
    }

    if(gimbal_cmd_send.autoaim_mode==AUTO_ON||gimbal_cmd_send.autoaim_mode==FIND_Enermy)
    {
        AutoAimSet();
        if(DataLebel.aim_flag!=1)
        {
            NoneAutoMouseControl();
        }
    }
    else
    {
        NoneAutoMouseControl();
    }
}

static void KeyControl()
{
    chassis_cmd_send.vx = (rc_data[TEMP].key[KEY_PRESS].w * 20000 - rc_data[TEMP].key[KEY_PRESS].s * 20000)*chassis_speed_buff; 
    chassis_cmd_send.vy = (rc_data[TEMP].key[KEY_PRESS].d * 20000 - rc_data[TEMP].key[KEY_PRESS].a * 20000)*chassis_speed_buff;

    ChassisRotateSet();
    switch (referee_data->GameRobotState.robot_level)
    {
    case 1:
        chassis_rotate_buff = 1;
        chassis_speed_buff  = 1;
        break;
    case 2:         
        chassis_rotate_buff = 1.2;
        chassis_speed_buff  = 1.03;
        break;
    case 3:
        chassis_rotate_buff = 1.3;
        chassis_speed_buff  = 1.05;
        break;
    case 4:
        chassis_rotate_buff = 1.4;
        chassis_speed_buff  = 1.1;
        break;
    case 5:
        chassis_rotate_buff = 1.5;
        chassis_speed_buff  = 1.15;
        break;
    case 6:
        chassis_rotate_buff = 1.6;
        chassis_speed_buff  = 1.2;
        break;
    case 7:
        chassis_rotate_buff = 1.7;
        chassis_speed_buff  = 1.25;
        break;
    case 8:
        chassis_rotate_buff = 1.8;
        chassis_speed_buff  = 1.3;
        break;
    case 9:
        chassis_rotate_buff = 1.9;
        chassis_speed_buff  = 1.35;
        break;
    case 10:
        chassis_rotate_buff = 2;
        chassis_speed_buff  = 1.4;
        break;
    default:
        chassis_rotate_buff = 1;
        chassis_speed_buff  = 1;
        break;
    }

    if(chassis_fetch_data.power_flag==1)
    {
        chassis_cmd_send.chassis_rotate_buff= 2;
    }
    else
    {
        chassis_cmd_send.chassis_rotate_buff= chassis_rotate_buff;
    }






    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) 
    {
    case 0:
        chassis_cmd_send.chassis_mode =CHASSIS_FOLLOW_GIMBAL_YAW;
        break;
    default:
        chassis_cmd_send.chassis_mode =CHASSIS_ROTATE;
    }

    if(rc_data[TEMP].key[KEY_PRESS].q)
    {
        DataLebel.reverse_flag=1;
        shoot_cmd_send.loader_mode = LOAD_REVERSE;
    }
    else
    {
        DataLebel.reverse_flag=0;
    }

    switch (rc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    {
    case 1:
        chassis_cmd_send.chassis_speed_buff= 2;
        break;
    default:
        break;
    }
}



/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    MouseControl();
    KeyControl();
}

/**
 * @brief 停止
 */
static void AnythingStop()
{
    gimbal_cmd_send.gimbal_mode=GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.loader_mode = LOAD_STOP;
    //重置与小电脑通信失败的标志位
    DataLebel.cmd_error_flag=0;
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
        BasicSet();
        MouseKeySet();   
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_right)) 
    {
        AnythingStop();
    }
}

static void EnemyJudge()
{
    if(referee_data->GameRobotState.robot_id>7)
    {
        minipc_send_data.Vision.detect_color = COLOR_RED;
    }
    else
    {
        minipc_send_data.Vision.detect_color = COLOR_BLUE;
    }
}
static void SendToUIData()
{
    ui_data.autoaim_mode=gimbal_cmd_send.autoaim_mode;
    ui_data.chassis_mode=chassis_cmd_send.chassis_mode;
    ui_data.loader_mode=shoot_cmd_send.loader_mode;
    ui_data.shoot_mode=shoot_cmd_send.shoot_mode;
    ui_data.gimbal_mode=gimbal_cmd_send.gimbal_mode;
}


/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    ControlDataDeal();

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // 推送消息,双板通信,视觉通信等
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    SendMinipcData(&minipc_send_data);
    SendToUIData();

}
