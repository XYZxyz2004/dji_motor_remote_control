/**
 * @file shoot.c
 * @author xyz
 * @brief ��ɷ�������ĳ�ʼ�������岦���Ͳ�ͬ�ٷ���
 * @version 1.1
 * @date 2024-09-24
 * @copyright Transistor BUAA
 */

#include "shoot.h"
 struct Struct_SHOOT_Manage_Object shoot_control={0};

 
 /**
 * @brief ���䴦���ʼ��
 * @param �ṹ���ַ
 */
void shoot_init(struct Struct_SHOOT_Manage_Object *shoot)
{
	(*shoot).pull_bullets_motor=motor_manage_object7;
	(*shoot).fric_wheel_l_motor=motor_manage_object5;
	(*shoot).fric_wheel_r_motor=motor_manage_object6;
//����2006�ٶȻ� 1.2
//�ȼ���BSP_PID_Init(&motor_manage_object7.v_pid_object,10,0.01,35,10000,16000,20);
BSP_PID_Init(&shoot->pull_bullets_motor.v_pid_object,10,0.01,35,10000,16000,20);
	
//����2006λ�û�	 1.2
//�ȼ���BSP_PID_Init(&motor_manage_object7.l_pid_object,0.5,0,5,1000,16000,20);
BSP_PID_Init(&shoot->pull_bullets_motor.l_pid_object,0.5,0,5,1000,16000,20);

//Ħ���ַ���3508�ٶȻ�
BSP_PID_Init(&shoot->fric_wheel_l_motor.v_pid_object,10,0.01,35,10000,16000,20);
BSP_PID_Init(&shoot->fric_wheel_l_motor.v_pid_object,10,0.01,35,10000,16000,20);
//�ȼ���	BSP_PID_Init(&motor_manage_object5.v_pid_object,10,0.01,35,10000,16000,20);
//�ȼ���BSP_PID_Init(&motor_manage_object6.v_pid_object,10,0.01,35,10000,16000,20);
	
shoot->pull_bullets_motor.motor_cotrol_way=location_control;
shoot->pull_bullets_motor.target_location=0.0f;	
	
shoot->fric_wheel_l_motor.motor_cotrol_way=velocity_control;
shoot->fric_wheel_l_motor.target_v=0.0f;
	
shoot->fric_wheel_r_motor.motor_cotrol_way=velocity_control;
shoot->fric_wheel_r_motor.target_v=0.0f;
	
shoot->fric_flag=0;
shoot->pull_bullets_state=3;
}

 /**
 * @brief ���ݴ�DBUS������s�ı仯����Ħ���ֵ��ٶȺ�ÿ�β�һ����
 * @param �ṹ���ַ
 */
void shoot_task(struct Struct_SHOOT_Manage_Object *shoot)
{
	if((uint8_t)((*local_rc_ctrl).rc.s[0])==1)
	{
		(*shoot).fric_wheel_l_motor.target_v=-3000.0f;
		(*shoot).fric_wheel_r_motor.target_v=3000.0f;
		(*shoot).fric_flag=1;
	}
		if((uint8_t)((*local_rc_ctrl).rc.s[0])==2)
	{
		(*shoot).fric_wheel_l_motor.target_v=-5000.0f;
		(*shoot).fric_wheel_r_motor.target_v=5000.0f;
		(*shoot).fric_flag=1;
	}
		if((uint8_t)((*local_rc_ctrl).rc.s[0])==3)
	{
		(*shoot).fric_wheel_l_motor.target_v=0.0f;
		(*shoot).fric_wheel_r_motor.target_v=0.0f;
		(*shoot).fric_flag=0;
	}
//ȷ��Ħ�������ٶ�ʱ���ı䲦������λ�ü����һ�β���	
if((*shoot).fric_flag==1&&(uint8_t)((*local_rc_ctrl).rc.s[1])!=(*shoot).pull_bullets_state)
{	(*shoot).pull_bullets_motor.target_location+=36*1.0/8;
	(*shoot).pull_bullets_state=(uint8_t)((*local_rc_ctrl).rc.s[1]);
}

}
