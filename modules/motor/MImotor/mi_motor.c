#include "mi_motor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

static uint8_t idx=0;
static MIMotorInstance *mi_motor_instance[MI_MOTOR_CNT] = {NULL}; // 会在control任务中遍历该指针数组进行pid计算

static CANInstance mi_sender_assignment[1] = {
    [0]={.can_handle=&hcan2, .txconf.IDE=CAN_ID_EXT, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0},},
    //[1]={.can_handle=&hcan2, .txconf.IDE=CAN_ID_EXT, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0} },
};

/**
 * @brief 6个用于确认是否有电机注册到mi_sender_assignment中的标志位,防止发送空帧,此变量将在DJIMotorControl()使用
 *        flag的初始化在 MotorSenderGrouping()中进行
 */
static uint8_t sender_enable_flag[2] = {0};

/**
  * @brief          float转int，数据打包用
  * @param[in]      x float数值
  * @param[in]      x_min float数值的最小值
  * @param[in]      x_max float数值的最大值
  * @param[in]      bits  int的数据位数
  * @retval         none
  */
static uint32_t FloatToUint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x=x_max;
    else if(x < x_min) x=x_min;
    return (uint32_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief          输入范围限制
  * @param[in]      x 输入数值
  * @param[in]      x_min 输入数值的最小值
  * @param[in]      x_max 输入数值的最大值
  * @retval         none
  */
static float RangeRestrict(float x, float x_min, float x_max)
{
    float res;
    if(x > x_max) res=x_max;
    else if(x < x_min) res=x_min;
    else res = x;
    return res;
}

/**
  * @brief          小米电机初始化
  * @param[out]     motor 电机结构体
  * @param[in]      phcan can总线句柄
  * @retval         none
  */
MIMotorInstance *MIMotorInit(Motor_Init_Config_s *config)
{
    MIMotorInstance *motor = (MIMotorInstance *)malloc(sizeof(MIMotorInstance));
    memset(motor, 0, sizeof(MIMotorInstance));
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);

    config->can_init_config.ext_flag=1;
    config->can_init_config.can_module_callback = DecodeMiMotor; 
    config->can_init_config.id = motor;                       
    motor->motor_can_instace = CANRegister(&config->can_init_config);
    mi_motor_instance[idx++] = motor;
    return motor;
}


/*-------------------- 按照小米电机文档写的各种通信类型 --------------------*/

/**
  * @brief          获取设备ID （通信类型0），需在电机使能前使用
  * @param[in]      motor 电机结构体
  * @retval         none
  */
void MI_motor_GetID(MIMotorInstance* motor)
{
    motor->motor_can_instace->EXT_ID.mode = 0;
    motor->motor_can_instace->EXT_ID.data = 0;
    motor->motor_can_instace->EXT_ID.motor_id = 0;
    motor->motor_can_instace->EXT_ID.res = 0;
 
    for(uint8_t i=0; i<8; i++)
    {
      motor->motor_can_instace->tx_buff[i]=0;
    }
    CANTransmit(&mi_sender_assignment[0], 5);
}

/**
  * @brief          运控模式电机控制指令（通信类型1）
  * @param[in]      motor 电机结构体
  * @param[in]      torque 目标力矩
  * @param[in]      MechPosition 
  * @param[in]      speed 
  * @param[in]      kp 
  * @param[in]      kd 
  * @retval         none
  */
void MI_motor_Control(MIMotorInstance* motor, float torque, float MechPosition , float speed , float kp , float kd)
{
    motor->motor_can_instace->EXT_ID.mode = 1;
    motor->motor_can_instace->EXT_ID.data = FloatToUint(torque,T_MIN,T_MAX,16) ;
    motor->motor_can_instace->EXT_ID.motor_id = 127;
    motor->motor_can_instace->EXT_ID.res = 0;

    motor->motor_can_instace->txconf.ExtId= *((uint32_t*)&(motor->motor_can_instace->EXT_ID));
    mi_sender_assignment[0].txconf.ExtId=motor->motor_can_instace->txconf.ExtId;
    mi_sender_assignment[0].tx_buff[0]=FloatToUint(MechPosition,P_MIN,P_MAX,16)>>8;
    mi_sender_assignment[0].tx_buff[1]=FloatToUint(MechPosition,P_MIN,P_MAX,16);
    mi_sender_assignment[0].tx_buff[2]=FloatToUint(speed,V_MIN,V_MAX,16)>>8;
    mi_sender_assignment[0].tx_buff[3]=FloatToUint(speed,V_MIN,V_MAX,16);
    mi_sender_assignment[0].tx_buff[4]=FloatToUint(kp,KP_MIN,KP_MAX,16)>>8;
    mi_sender_assignment[0].tx_buff[5]=FloatToUint(kp,KP_MIN,KP_MAX,16);
    mi_sender_assignment[0].tx_buff[6]=FloatToUint(kd,KD_MIN,KD_MAX,16)>>8;
    mi_sender_assignment[0].tx_buff[7]=FloatToUint(kd,KD_MIN,KD_MAX,16);
}

/**
  * @brief          小米电机反馈帧解码（通信类型2）
  * @param[in]      Rx_can_info 接受到的电机数据结构体
  * @param[in]      rx_data[8] CAN线接收到的数据
  * @note           将接收到的CAN线数据解码到电机数据结构体中
  * @retval         none
  */
void DecodeMiMotor(CANInstance *_instance)
{
    uint8_t *rxbuff = _instance->rx_buff;
    MIMotorInstance *motor = (MIMotorInstance *)_instance->id;

    uint16_t decode_temp_mi;//小米电机反馈数据解码缓冲
    decode_temp_mi = (rxbuff[0] << 8 | rxbuff[1]);
    _instance->RxCAN_info.angle = ((float)decode_temp_mi-32767.5)/32767.5*4*3.1415926f;;

    decode_temp_mi = (rxbuff[2] << 8 | rxbuff[3]);
     _instance->RxCAN_info.speed = ((float)decode_temp_mi-32767.5)/32767.5*30.0f;

    decode_temp_mi = (rxbuff[4] << 8 | rxbuff[5]);
     _instance->RxCAN_info.torque = ((float)decode_temp_mi-32767.5)/32767.5*12.0f;

    decode_temp_mi = (rxbuff[6] << 8 | rxbuff[7]);
     _instance->RxCAN_info.temperature = (float)decode_temp_mi/10.0f;
}


/**
  * @brief          小米电机使能（通信类型 3）
  * @param[in]      motor 电机结构体
  * @param[in]      id 电机id
  * @retval         none
  */
void MIMotorEnable(MIMotorInstance* motor)
{
    motor->motor_can_instace->EXT_ID.mode = 3;
    motor->motor_can_instace->EXT_ID.data = 1;
    motor->motor_can_instace->EXT_ID.motor_id = 127;
    motor->motor_can_instace->EXT_ID.res = 0;
    motor->motor_can_instace->txconf.ExtId= *((uint32_t*)&(motor->motor_can_instace->EXT_ID));
    mi_sender_assignment[0].txconf.ExtId=motor->motor_can_instace->txconf.ExtId;

    mi_sender_assignment[0].tx_buff[0]=0;
    mi_sender_assignment[0].tx_buff[1]=0;
    mi_sender_assignment[0].tx_buff[2]=0;
    mi_sender_assignment[0].tx_buff[3]=0;
    mi_sender_assignment[0].tx_buff[4]=0;
    mi_sender_assignment[0].tx_buff[5]=0;
    mi_sender_assignment[0].tx_buff[6]=0;
    mi_sender_assignment[0].tx_buff[7]=0;
}

/**
  * @brief          电机停止运行帧（通信类型4）
  * @param[in]      motor 电机结构体
  * @retval         none
  */
void MIMotorInstancetop(MIMotorInstance* motor)
{
    motor->motor_can_instace->EXT_ID.mode = 4;
    motor->motor_can_instace->EXT_ID.motor_id =127;
    motor->motor_can_instace->EXT_ID.data = 1;
    motor->motor_can_instace->EXT_ID.res = 0;
    motor->motor_can_instace->txconf.ExtId= *((uint32_t*)&(motor->motor_can_instace->EXT_ID));
    mi_sender_assignment[0].txconf.ExtId=motor->motor_can_instace->txconf.ExtId;
    mi_sender_assignment[0].tx_buff[0]=1;
    mi_sender_assignment[0].tx_buff[1]=0;
    mi_sender_assignment[0].tx_buff[2]=0;
    mi_sender_assignment[0].tx_buff[3]=0;
    mi_sender_assignment[0].tx_buff[4]=0;
    mi_sender_assignment[0].tx_buff[5]=0;
    mi_sender_assignment[0].tx_buff[6]=0;
    mi_sender_assignment[0].tx_buff[7]=0;
}


/**
  * @brief          设置电机机械零位（通信类型6）会把当前电机位置设为机械零位（掉电丢失）
  * @param[in]      motor 电机结构体
  * @retval         none
  */
void MIMotorInstanceetMechPositionToZero(MIMotorInstance* motor)
{
    motor->motor_can_instace->EXT_ID.mode = 6;
    motor->motor_can_instace->EXT_ID.motor_id =127;
    motor->motor_can_instace->EXT_ID.data = 1;
    motor->motor_can_instace->EXT_ID.res = 0;

    motor->motor_can_instace->txconf.ExtId= *((uint32_t*)&(motor->motor_can_instace->EXT_ID));
    mi_sender_assignment[0].txconf.ExtId=motor->motor_can_instace->txconf.ExtId;
    mi_sender_assignment[0].tx_buff[0]=1;
    mi_sender_assignment[0].tx_buff[1]=0;
    mi_sender_assignment[0].tx_buff[2]=0;
    mi_sender_assignment[0].tx_buff[3]=0;
    mi_sender_assignment[0].tx_buff[4]=0;
    mi_sender_assignment[0].tx_buff[5]=0;
    mi_sender_assignment[0].tx_buff[6]=0;
    mi_sender_assignment[0].tx_buff[7]=0;
}

/**
  * @brief          设置电机CAN_ID（通信类型7）更改当前电机CAN_ID , 立即生效，需在电机使能前使用
  * @param[in]      motor 电机结构体
  * @param[in]      Now_ID 电机现在的ID
  * @param[in]      Target_ID 想要改成的电机ID
  * @retval         none
  */
void MI_motor_ChangeID(MIMotorInstance* motor,uint8_t Now_ID,uint8_t Target_ID)
{
    motor->motor_id = Now_ID;

    motor->motor_can_instace->EXT_ID.mode = 7;	
    motor->motor_can_instace->EXT_ID.motor_id = Now_ID;
    motor->motor_can_instace->EXT_ID.data = Target_ID << 8 | 1;
    motor->motor_can_instace->EXT_ID.res = 0;
 
    for(uint8_t i=0; i<8; i++)
    {
        motor->motor_can_instace->tx_buff[i]=0;
    }
}


/**
  * @brief          单个参数读取（通信类型17）
  * @param[in]      motor 电机结构体
  * @param[in]      index 功能码
  * @retval         none
  */
void MI_motor_ReadParam(MIMotorInstance* motor,uint16_t index)
{
    motor->motor_can_instace->EXT_ID.mode = 17;
    motor->motor_can_instace->EXT_ID.motor_id =127;
    motor->motor_can_instace->EXT_ID.data = 1;
    motor->motor_can_instace->EXT_ID.res = 0;
    
    memcpy(&motor->motor_can_instace->tx_buff[0],&index,2);

    for(uint8_t i=2; i<8; i++)
    {
        motor->motor_can_instace->tx_buff[i]=0;
    }
}


/**
  * @brief          小米电机运行模式切换
  * @param[in]      motor 电机结构体
  * @param[in]      run_mode 更改的模式
  * @note           通信类型18 （掉电丢失）
  * @retval         none
  */
void MIMotorModeSwitch(MIMotorInstance* motor, uint8_t run_mode)
{
    uint16_t index = 0X7005;

    motor->motor_can_instace->EXT_ID.mode = 18;
    motor->motor_can_instace->EXT_ID.motor_id =127;
    motor->motor_can_instace->EXT_ID.data = 1;
    motor->motor_can_instace->EXT_ID.res = 0;

    for(uint8_t i=0;i<8;i++){
      motor->motor_can_instace->tx_buff[i]=0;
    }

    memcpy(&motor->motor_can_instace->tx_buff[0],&index,2);
    memcpy(&motor->motor_can_instace->tx_buff[4],&run_mode, 1);
}


/**
  * @brief          小米电机控制参数写入
  * @param[in]      motor 电机结构体
  * @param[in]      index 功能码
  * @param[in]      param 写入的参数
  * @note           通信类型18 （掉电丢失）
  * @retval         none
  */
 void MI_motor_WritePram(MIMotorInstance* motor, uint16_t index, float param)
 {
    motor->motor_can_instace->EXT_ID.mode = 18;
    motor->motor_can_instace->EXT_ID.motor_id =127;
    motor->motor_can_instace->EXT_ID.data = 1;
    motor->motor_can_instace->EXT_ID.res = 0;

    memcpy(&motor->motor_can_instace->tx_buff[0],&index,2);
    motor->motor_can_instace->tx_buff[2]=0;
    motor->motor_can_instace->tx_buff[3]=0;
    memcpy(&motor->motor_can_instace->tx_buff[4],&param, 4);

  }

void MIMotorSetPid(MIMotorInstance* motor, float location_kp,float limit_speed,float speed_kp,float speed_ki)
{
    MI_motor_WritePram(motor,0x7017,limit_speed);
    MI_motor_WritePram(motor,0x701E,location_kp);
    MI_motor_WritePram(motor,0x701F,speed_kp);
    MI_motor_WritePram(motor,0x7020,speed_ki);
}

void MiMotorSetRef(MIMotorInstance* motor,float location_ref)
{
    MI_motor_WritePram(motor,0x7016,location_ref);
}
/*-------------------- 封装的一些控制函数 --------------------*/

/**
  * @brief          小米电机力矩控制模式控制指令
  * @param[in]      motor 电机结构体
  * @param[in]      torque 目标力矩
  * @retval         none
  */
void MI_motor_TorqueControl(MIMotorInstance* motor, float torque)
{
  MI_motor_Control(motor, torque, 0, 0, 0, 0);
}

/**
  * @brief          小米电机位置模式控制指令
  * @param[in]      motor 电机结构体
  * @param[in]      location 控制位置 rad
  * @param[in]      kp 响应速度(到达位置快慢)，一般取1-10
  * @param[in]      kd 电机阻尼，过小会震荡，过大电机会震动明显。一般取0.5左右
  * @retval         none
  */
void MI_motor_LocationControl(MIMotorInstance* motor, float location, float kp, float kd)
{
  MI_motor_Control(motor, 0, location, 0, kp, kd);
}

/**
  * @brief          小米电机速度模式控制指令
  * @param[in]      motor 电机结构体
  * @param[in]      speed 控制速度
  * @param[in]      kd 响应速度，一般取0.1-1
  * @retval         none
  */
void MIMotorInstancepeedControl(MIMotorInstance* motor, float speed, float kd)
{
    MI_motor_Control(motor, 0, 0, speed, 0, kd);
}

void MiMotorControl()
{
    CANTransmit(&mi_sender_assignment[0], 1);
}