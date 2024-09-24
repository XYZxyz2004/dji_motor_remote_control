/**
 * @file task_init.c
 * @author xyz
 * @brief 完成相关任务的板级支持包启动
 * @version 1.1
 * @date 2024-09-24
 * @copyright Transistor BUAA
 */
#include "task_init.h"
void task_start_init()
 {
	 //遥控器启动
	 	 remote_control_init();
	 
	 //串口发送绘图启动
    usart1_tx_dma_init();
//初始化各电机
  bsp_motor_init(&motor_manage_object1,0x201);
  bsp_motor_init(&motor_manage_object2,0x202);
  bsp_motor_init(&motor_manage_object3,0x203);
  bsp_motor_init(&motor_manage_object4,0x204);
  bsp_motor_init(&motor_manage_object5,0x205);
  bsp_motor_init(&motor_manage_object6,0x206);
  bsp_motor_init(&motor_manage_object7,0x207);
  bsp_motor_init(&motor_manage_object8,0x208);
	bsp_motor_init(&motor_manage_object9,0x209);
	bsp_motor_init(&motor_manage_objectA,0x20A);
	bsp_motor_init(&motor_manage_objectB,0x20B);

 }