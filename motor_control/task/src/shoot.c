/**
 * @file shoot.c
 * @author xyz
 * @brief 完成发射任务的初始化及具体拨弹和不同速发射
 * @version 1.1
 * @date 2024-09-24
 * @copyright Transistor BUAA
 */

#include "shoot.h"
 struct Struct_SHOOT_Manage_Object shoot_control={0};

 
 /**
 * @brief 发射处理初始化
 * @param 结构体地址
 */
void shoot_init(struct Struct_SHOOT_Manage_Object *shoot)
{
	(*shoot).pull_bullets_motor=motor_manage_object7;
	(*shoot).fric_wheel_l_motor=motor_manage_object5;
	(*shoot).fric_wheel_r_motor=motor_manage_object6;
//拨弹2006速度环 1.2
//等价于BSP_PID_Init(&motor_manage_object7.v_pid_object,10,0.01,35,10000,16000,20);
BSP_PID_Init(&shoot->pull_bullets_motor.v_pid_object,10,0.01,35,10000,16000,20);
	
//拨弹2006位置环	 1.2
//等价于BSP_PID_Init(&motor_manage_object7.l_pid_object,0.5,0,5,1000,16000,20);
BSP_PID_Init(&shoot->pull_bullets_motor.l_pid_object,0.5,0,5,1000,16000,20);

//摩擦轮发射3508速度环
BSP_PID_Init(&shoot->fric_wheel_l_motor.v_pid_object,10,0.01,35,10000,16000,20);
BSP_PID_Init(&shoot->fric_wheel_l_motor.v_pid_object,10,0.01,35,10000,16000,20);
//等价于	BSP_PID_Init(&motor_manage_object5.v_pid_object,10,0.01,35,10000,16000,20);
//等价于BSP_PID_Init(&motor_manage_object6.v_pid_object,10,0.01,35,10000,16000,20);
	
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
 * @brief 根据大疆DBUS控制器s的变化控制摩擦轮的速度和每次拨一个弹
 * @param 结构体地址
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
//确保摩擦轮有速度时，改变拨动开关位置即完成一次拨弹	
if((*shoot).fric_flag==1&&(uint8_t)((*local_rc_ctrl).rc.s[1])!=(*shoot).pull_bullets_state)
{	(*shoot).pull_bullets_motor.target_location+=36*1.0/8;
	(*shoot).pull_bullets_state=(uint8_t)((*local_rc_ctrl).rc.s[1]);
}

}
