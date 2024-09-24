/**
 * @file task_init.c
 * @author xyz
 * @brief ����������İ弶֧�ְ�����
 * @version 1.1
 * @date 2024-09-24
 * @copyright Transistor BUAA
 */
#include "task_init.h"
void task_start_init()
 {
	 //ң��������
	 	 remote_control_init();
	 
	 //���ڷ��ͻ�ͼ����
    usart1_tx_dma_init();
//��ʼ�������
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