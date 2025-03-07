#include "referee_task.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "cmsis_os.h"

static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data); // 模式切换检测
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data); // 测试用函数，实现模式自动变化

referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    referee_recv_info = RefereeInit(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    Interactive_data = UI_data;                            // 获取UI绘制需要的机器人状态数据
    referee_recv_info->init_flag = 1;
    return referee_recv_info;
}

void UITask()
{
    MyUIRefresh(referee_recv_info, Interactive_data);
}

static Graph_Data_t UI_shoot_line[10]; // 射击准线
static Graph_Data_t UI_Energy[3];      // 电容能量条
static String_Data_t UI_State_sta[12];  // 机器人状态,静态只需画一次
static String_Data_t UI_State_dyn[6];  // 机器人状态,动态先add才能change
static uint32_t shoot_line_location[10] = {960,400,540, 440, 420,400,380,360,340,320};
static Graph_Data_t UI_State_Cir[5];
static Graph_Data_t UI_State_Rec[5];
void MyUIInit()
{
    if (!referee_recv_info->init_flag)
        vTaskDelete(NULL); // 如果没有初始化裁判系统则直接删除ui任务
    while (referee_recv_info->GameRobotState.robot_id == 0)
        osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查

    DeterminRobotID();                                            // 确定ui要发送到的目标客户端
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

    // 绘制发射基准线 955
    UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_White, 2, shoot_line_location[0],540, shoot_line_location[0], 320);
    UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 930, shoot_line_location[1], 990, shoot_line_location[1]);
    UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_White, 2, 860, shoot_line_location[2], 1060, shoot_line_location[2]);
    UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 900, shoot_line_location[3], 1020, shoot_line_location[3]);
    UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 930, shoot_line_location[4], 990, shoot_line_location[4]);
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3], UI_shoot_line[4]);
    UILineDraw(&UI_shoot_line[5], "sl5", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 930, shoot_line_location[5], 990, shoot_line_location[5]);
    UILineDraw(&UI_shoot_line[6], "sl6", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 930, shoot_line_location[6], 990, shoot_line_location[6]);
    UILineDraw(&UI_shoot_line[7], "sl7", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 950, shoot_line_location[7], 970, shoot_line_location[7]);
    UILineDraw(&UI_shoot_line[8], "sl8", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 950, shoot_line_location[8], 970, shoot_line_location[8]);
    UILineDraw(&UI_shoot_line[9], "sl9", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 950, shoot_line_location[9], 970, shoot_line_location[9]);
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_shoot_line[5], UI_shoot_line[6], UI_shoot_line[7], UI_shoot_line[8], UI_shoot_line[9]);

    UICircleDraw(&UI_State_Cir[0],"sa0",UI_Graph_ADD,9,UI_Color_White,5,130,740,10);
    UICircleDraw(&UI_State_Cir[1],"sa1",UI_Graph_ADD,9,UI_Color_White,5,130,690,10);
    UICircleDraw(&UI_State_Cir[2],"sa2",UI_Graph_ADD,9,UI_Color_White,5,130,640,10);
    UICircleDraw(&UI_State_Cir[3],"sa3",UI_Graph_ADD,9,UI_Color_White,5,130,590,10);
    UICircleDraw(&UI_State_Cir[4],"sa4",UI_Graph_ADD,9,UI_Color_White,5,130,540,10);
    UIGraphRefresh(&referee_recv_info->referee_id,5,UI_State_Cir[0],UI_State_Cir[1], UI_State_Cir[2],UI_State_Cir[3],UI_State_Cir[4]);


    // 绘制车辆状态标志指示
    UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 750, "chassis:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[0]);
    UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 700, "gimbal:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);
    UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 650, "shoot:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[2]);
    UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 600, "loader:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[3]);
    UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 6, UI_Color_White, 15, 2, 150, 550, "autoaim:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[4]);

    // 绘制车辆状态标志指示
    UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 8, UI_Color_Yellow, 7, 2, 1070, 440, "2m");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[5]);
    UICharDraw(&UI_State_sta[6], "ss6", UI_Graph_ADD, 8, UI_Color_Yellow, 7, 2, 1070, 400, "3m");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[6]);
    UICharDraw(&UI_State_sta[7], "ss7", UI_Graph_ADD, 8, UI_Color_Yellow, 7, 2, 1070, 320, "4m");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[7]);

    // 绘制车辆状态标志，动态
    // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
    UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 750, "off      ");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 700, "off      ");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 650, "off      ");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 600, "off      ");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_ADD, 6, UI_Color_White, 15, 2, 270, 550, "off      ");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);

    UIRectangleDraw(&UI_State_Rec[0], "sr0", UI_Graph_ADD, 6, UI_Color_White,3,600,300,1320,800);
    UIGraphRefresh(&referee_recv_info->referee_id,1, UI_State_Rec[0]);
}

// 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
static uint8_t count = 0;
static uint16_t count1 = 0;
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data) // 测试用函数，实现模式自动变化
{
    count++;
    if (count >= 60)
    {
        count = 0;
        count1++;
    }
    switch (count1 % 4)
    {
    case 0:
    {
        _Interactive_data->chassis_mode = CHASSIS_ZERO_FORCE;
        _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
        _Interactive_data->shoot_mode = SHOOT_ON;
        _Interactive_data->loader_mode = LOAD_BURSTFIRE;
        _Interactive_data->autoaim_mode = AUTO_OFF;
    }
    case 1:
    {
        _Interactive_data->chassis_mode = CHASSIS_ROTATE;
        _Interactive_data->gimbal_mode = GIMBAL_GYRO_MODE;
        _Interactive_data->shoot_mode = SHOOT_OFF;
        _Interactive_data->loader_mode = LOAD_1_BULLET;
        _Interactive_data->autoaim_mode = AUTO_ON;

        break;
    }
    case 2:
    {
        _Interactive_data->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        _Interactive_data->gimbal_mode = GIMBAL_GYRO_MODE;
        _Interactive_data->shoot_mode = SHOOT_ON;
        _Interactive_data->loader_mode = LOAD_STOP;
        _Interactive_data->autoaim_mode = FIND_Enermy;

        break;
    }
    case 3:
    {
        _Interactive_data->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
        _Interactive_data->shoot_mode = SHOOT_OFF;
        _Interactive_data->loader_mode = LOAD_REVERSE;
        _Interactive_data->autoaim_mode = FIND_Enermy;

        break;
    }
    default:
        break;
    }
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    UIChangeCheck(_Interactive_data);
    // chassis
    if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
    {
        switch (_Interactive_data->chassis_mode)
        {
            case CHASSIS_ZERO_FORCE:
                UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_White, 15, 2, 270, 750, "off      ");
                UICircleDraw(&UI_State_Cir[0],"sa0",UI_Graph_Change,9,UI_Color_White,5,130,740,10);
                break;
            case CHASSIS_ROTATE:
                UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Purplish_red, 15, 2, 270, 750, "rotate   ");
                UICircleDraw(&UI_State_Cir[0],"sa0",UI_Graph_Change,9,UI_Color_Purplish_red,5,130,740,10);
                // 此处注意字数对齐问题，字数相同才能覆盖掉
                break;
            case CHASSIS_FOLLOW_GIMBAL_YAW:
                UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Green, 15, 2, 270, 750, "follow   ");
                UICircleDraw(&UI_State_Cir[0],"sa0",UI_Graph_Change,9,UI_Color_Green,5,130,740,10);
                break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
        UIGraphRefresh(&referee_recv_info->referee_id,1,UI_State_Cir[0]);
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    }

    // gimbal
    if (_Interactive_data->Referee_Interactive_Flag.gimbal_flag == 1)
    {
        switch (_Interactive_data->gimbal_mode)
        {
            case GIMBAL_ZERO_FORCE:
            {
                UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_White, 15, 2, 270, 700, "off      ");
                UICircleDraw(&UI_State_Cir[1],"sa1",UI_Graph_Change,9,UI_Color_White,5,130,690,10);

                break;
            }
            case GIMBAL_GYRO_MODE:
            {
                UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Green, 15, 2, 270, 700, "normal   ");
                UICircleDraw(&UI_State_Cir[1],"sa1",UI_Graph_Change,9,UI_Color_Green,5,130,690,10);

                break;
            }
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
        UIGraphRefresh(&referee_recv_info->referee_id,1,UI_State_Cir[1]);

        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 0;
    }
    // shoot
    if (_Interactive_data->Referee_Interactive_Flag.shoot_flag == 1)
    {
        switch (_Interactive_data->shoot_mode)
        {
            case SHOOT_OFF:
            {
                UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_White, 15, 2, 270, 650, "off      ");
                UICircleDraw(&UI_State_Cir[2],"sa2",UI_Graph_Change,9,UI_Color_White,5,130,640,10);

                break;
            }
            case SHOOT_ON:
            {
                UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Green, 15, 2, 270, 650, "normal   ");
                UICircleDraw(&UI_State_Cir[2],"sa2",UI_Graph_Change,9,UI_Color_Green,5,130,640,10);

                break;
            }
        }        
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
        UIGraphRefresh(&referee_recv_info->referee_id,1,UI_State_Cir[2]);
        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
    }
    //loader
    if (_Interactive_data->Referee_Interactive_Flag.loader_flag == 1)
    {
        switch (_Interactive_data->loader_mode)
        {
            case LOAD_1_BULLET:
            {
                UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Green, 15, 2, 270, 600, "normal   ");
                UICircleDraw(&UI_State_Cir[3],"sa3",UI_Graph_Change,9,UI_Color_Green,5,130,595,10);
                break;
            }
            case LOAD_BURSTFIRE:
            {
                UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Purplish_red, 15, 2, 270, 600, "angry    ");
                UICircleDraw(&UI_State_Cir[3],"sa3",UI_Graph_Change,9,UI_Color_Purplish_red,5,130,595,10);
                break;
            }
            case LOAD_REVERSE:
            {
                UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 600, "reverse  ");
                UICircleDraw(&UI_State_Cir[3],"sa3",UI_Graph_Change,9,UI_Color_Main,5,130,595,10);

                break;
            }
            case LOAD_STOP:
            {
                UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_White, 15, 2, 270, 600, "off      ");
                UICircleDraw(&UI_State_Cir[3],"sa3",UI_Graph_Change,9,UI_Color_White,5,130,595,10);
                break;                
            }
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
        UIGraphRefresh(&referee_recv_info->referee_id,1,UI_State_Cir[3]);
        _Interactive_data->Referee_Interactive_Flag.loader_flag = 0;
    }

    if (_Interactive_data->Referee_Interactive_Flag.aim_flag == 1)
    {
        switch (_Interactive_data->autoaim_mode)
        {
            case AUTO_OFF:
            {
                UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 6, UI_Color_White, 15, 2, 270, 550, "off      ");
                UICircleDraw(&UI_State_Cir[4],"sa4",UI_Graph_Change,6,UI_Color_White,5,130,540,10);
                UIRectangleDraw(&UI_State_Rec[0],"sr0",UI_Graph_Change,6,UI_Color_White,3,600,300,1320,800);
                break;
            }
            case FIND_Enermy:
            {
                UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 6, UI_Color_Yellow, 15, 2, 270, 550, "find     ");
                UICircleDraw(&UI_State_Cir[4],"sa4",UI_Graph_Change,6,UI_Color_Yellow,5,130,540,10);
                UIRectangleDraw(&UI_State_Rec[0],"sr0",UI_Graph_Change,6,UI_Color_Yellow,3,600,300,1320,800);
                break;
            }
            case AUTO_ON:
            {
                UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 6, UI_Color_Main, 15, 2, 270, 550, "on       ");
                UICircleDraw(&UI_State_Cir[4],"sa4",UI_Graph_Change,6,UI_Color_Main,5,130,540,10);
                UIRectangleDraw(&UI_State_Rec[0],"sr0",UI_Graph_Change,6,UI_Color_Main,3,600,300,1320,800);
                break;
            }
        }        
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
        UIGraphRefresh(&referee_recv_info->referee_id,1, UI_State_Cir[4]);
        UIGraphRefresh(&referee_recv_info->referee_id,1,UI_State_Rec[0]);
        _Interactive_data->Referee_Interactive_Flag.aim_flag = 0;
    }
}

/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }

    if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
        _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
    }

    if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
        _Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
    }

    if (_Interactive_data->loader_mode != _Interactive_data->loader_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.loader_flag = 1;
        _Interactive_data->loader_last_mode = _Interactive_data->loader_mode;
    }

    if (_Interactive_data->autoaim_mode != _Interactive_data->autoaim_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.aim_flag = 1;
        _Interactive_data->autoaim_last_mode = _Interactive_data->autoaim_mode;
    }
}
