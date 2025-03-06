#include "super_cap.h"
#include "memory.h"
#include "stdlib.h"

static SuperCapInstance *super_cap_instance = NULL; // 可以由app保存此指针

/*
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s* supercap_config);
void SuperCapSend(SuperCapInstance *instance, uint8_t *data);
```
## 私有函数和变量

```c
static SuperCapInstance *super_cap_instance = NULL;
static uint8_t *rxbuff;
static void SuperCapRxCallback(can_instance *_instance)
```

`SuperCapRxCallback()`是super cap初始化can实例时的回调函数，用于can接收中断，进行协议解析。

## 使用范例

初始化时设置如下：

```c
SuperCap_Init_Config_s capconfig = {
		.can_config = {
			.can_handle = &hcan1,
			.rx_id = 0x301,
			.tx_id = 0x302
		},
		.recv_data_len = 4*sizeof(uint16_t),
		.send_data_len = sizeof(uint8_t)
	};
SuperCapInstance *ins =SuperCapInit(&capconfig);
```


发送通过`SuperCapSend()`，建议使用强制类型转换：

```c
uint16_t tx = 0x321;
SuperCapSend(ins, (uint8_t*)&tx);
```

*/


















static void SuperCapRxCallback(CANInstance *_instance)
{
    uint8_t *rxbuff;
    SuperCap_Msg_s *Msg;
    rxbuff = _instance->rx_buff;
    Msg = &super_cap_instance->cap_msg;
    Msg->vol = (int16_t)(rxbuff[0] << 8 | rxbuff[1]);
}

SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config)
{
    super_cap_instance = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
    memset(super_cap_instance, 0, sizeof(SuperCapInstance));
    
    supercap_config->can_config.can_module_callback = SuperCapRxCallback;
    super_cap_instance->can_ins = CANRegister(&supercap_config->can_config);
    return super_cap_instance;
}

void SuperCapSend(SuperCapInstance *instance, uint8_t *data)
{
    memcpy(instance->can_ins->tx_buff, data, 8);
    CANTransmit(instance->can_ins,1);
}

SuperCap_Msg_s SuperCapGet(SuperCapInstance *instance)
{
    return instance->cap_msg;
}