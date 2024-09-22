/**
 * @file bsp_pid.h
 * @author xyz
 * @brief pid相关处理
 * @version 1.1
 * @date 2024-09-18
 * @copyright Transistor BUAA
 */

#ifndef BSP_PID_H
#define BSP_PID_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* Exported macros -----------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/

extern  uint8_t pset;
extern uint8_t iset;
extern uint8_t dset;

/**
 * @brief PID处理结构体
 */
struct Struct_PID_Manage_Object
{
   float kp;
   float ki;
   float kd;

   float error;
   float last_error;
		float before_last_error;
   float integral_error;//误差积分累积
	
   float set_point;//目标值
   float now_point;//当前值

   float integral_limit;//积分限幅
   float output_limit;//输出限幅
	float deadzone;//死区
 
   float p_out;//p输出
   float i_out;//i输出
   float d_out;//d输出
   float output;//总输出
	

};

extern struct Struct_PID_Manage_Object pid1;
extern struct Struct_PID_Manage_Object pid2;
extern struct Struct_PID_Manage_Object pid3;
extern struct Struct_PID_Manage_Object pid4;
extern struct Struct_PID_Manage_Object pid5;
extern struct Struct_PID_Manage_Object pid6;
extern struct Struct_PID_Manage_Object pid7;
extern struct Struct_PID_Manage_Object pid8;
extern struct Struct_PID_Manage_Object pid9;
extern struct Struct_PID_Manage_Object pid10;
extern struct Struct_PID_Manage_Object pid11;
extern struct Struct_PID_Manage_Object pid12;
extern struct Struct_PID_Manage_Object pid13;
extern struct Struct_PID_Manage_Object pid14;
extern struct Struct_PID_Manage_Object pid15;
extern struct Struct_PID_Manage_Object pid16;

/* Exported variables --------------------------------------------------------*/
extern void BSP_PID_Init(struct Struct_PID_Manage_Object *pid,float _kp,float _ki,float _kd,float _integral_limit,float _output_limit,float _deadzone);
extern float BSP_PID_Model1_Update(struct Struct_PID_Manage_Object *pid,float _now_point,float _set_point);
extern float BSP_PID_Model2_Update(struct Struct_PID_Manage_Object *pid,float _now_point,float _set_point);
extern void pid_change(struct Struct_PID_Manage_Object *pid,float _kp,float _ki,float _kd);
#endif
