#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"

static Minipc_Recv_s minipc_recv_data;
static Minipc_Send_s minipc_send_data;
static DaemonInstance *minipc_daemon_instance;

void VisionSetFlag(uint8_t color)
{
    minipc_send_data.Vision.detect_color=color;
}

void VisionSetAltitude(uint8_t color)
{
}

static USARTInstance *minipc_usart_instance;

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(minipc_usart_instance);
#endif // !VISION_USE_UART
    LOGWARNING("[vision] vision offline, restart communication.");
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"


/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeMinpc()
{
    uint16_t flag_register;
    DaemonReload(minipc_daemon_instance); // 喂狗
    get_protocol_info_vision(minipc_usart_instance->recv_buff, &flag_register,&minipc_recv_data);
}

Minipc_Recv_s *minipcInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeMinpc;
    conf.recv_buff_size = Minipc_Recv_sIZE;
    conf.usart_handle = _handle;
    minipc_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = minipc_usart_instance,
        .reload_count = 10,
    };
    minipc_daemon_instance = DaemonRegister(&daemon_conf);

    return &minipc_recv_data;
}


/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void SendMinipcData()
{
    // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
    // 析构后的陷阱需要特别注意!
    static uint16_t flag_register;
    static uint8_t send_buff[Minipc_Send_sIZE];
    static uint16_t tx_len;
    // TODO: code to set flag_register
    flag_register = 30 << 8 | 0b00000001;
    // 将数据转化为seasky协议的数据包
    get_protocol_send_Vision_data(0x02, flag_register, &minipc_send_data, 1, send_buff, &tx_len);

    USARTSend(minipc_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA); // 和视觉通信使用IT,防止和接收使用的DMA冲突
    // 此处为HAL设计的缺陷,DMASTOP会停止发送和接收,导致再也无法进入接收中断.
    // 也可在发送完成中断中重新启动DMA接收,但较为复杂.因此,此处使用IT发送.
    // 若使用了daemon,则也可以使用DMA发送.
}

#endif // VISION_USE_UART