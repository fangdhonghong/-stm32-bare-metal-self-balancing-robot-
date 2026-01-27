#ifndef __BLUETOOTHCONTROL_H
#define __BLUETOOTHCONTROL_H

#include "stm32f1xx_hal.h"
#include "usart.h"
#include "pid.h"    // 必须包含，用于访问 PID 结构体

// 定义摇杆数据结构体
typedef struct {
    int16_t speed;  // 速度
    int16_t turn;   // 转向
} Joystick_TypeDef;

extern uint32_t last_heartbeat_tick; // 上次心跳时间戳

void Bluetooth_Parse_Binary(uint8_t *data);

#endif
