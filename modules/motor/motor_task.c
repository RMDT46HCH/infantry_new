#include "motor_task.h"
#include "dji_motor.h"
#include "servo_motor.h"
#include "mi_motor.h"
void MotorControlTask()
{
    static uint8_t cnt = 0; 
    if(cnt%5==0) //200hz
    DJIMotorControl();
    if(cnt%10==0) //100hz
    MiMotorControl();

    ServeoMotorControl();
}
