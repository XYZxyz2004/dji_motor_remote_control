/**
 * @file bsp_motor.h
 * @author xyz
 * @brief motor相关处理
 * @version 1.1
 * @date 2024-09-18
 * @copyright Transistor BUAA
 */


#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* Exported macros -----------------------------------------------------------*/
#include "bsp_pid.h"
#include "bsp_can.h"
/* Exported types ------------------------------------------------------------*/

//速度环/位置环选择
enum velocity_location_contorl
{
	velocity_control,
	location_control
	
};
//pid控制方式选择
enum pid_control
{
	pid_control1,
	pid_control2
};
	

/**
* @brief 电机处理结构体
 */
struct Struct_MOTOR_Manage_Object
{
	//经Rx_Buffer中的数据转换得到的电机基础数据
int16_t omega;
uint16_t encoder;
int16_t torque;
uint8_t temperaure;
uint16_t can_id;
	
int16_t i_send;//发送电流/电压
	uint16_t pre_encoder;//上次的编码器位置
uint16_t now_encoder;//这次的编码器位置
int32_t  delta_encoder;//编码器变化
int32_t total_encoder;//当前总共已经变换的编码器
int32_t total_round;//旋转圈数
	
int16_t target_v;//目标速度
float target_location;//目标位置
	enum velocity_location_contorl motor_cotrol_way;
struct  Struct_PID_Manage_Object v_pid_object;//对应速度环pid处理结构体
	struct  Struct_PID_Manage_Object l_pid_object;//对应角度环pid处理结构体
};

extern struct Struct_MOTOR_Manage_Object motor_manage_object1;
extern struct Struct_MOTOR_Manage_Object motor_manage_object2;
extern struct Struct_MOTOR_Manage_Object motor_manage_object3;
extern struct Struct_MOTOR_Manage_Object motor_manage_object4;
//默认5，6对应左右摩擦轮3508电机  7对应拨弹2006电机，不要更改！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
extern struct Struct_MOTOR_Manage_Object motor_manage_object5;
extern struct Struct_MOTOR_Manage_Object motor_manage_object6;
extern struct Struct_MOTOR_Manage_Object motor_manage_object7;
extern struct Struct_MOTOR_Manage_Object motor_manage_object8;
//默认9，A对应 yaw pitch 6020电机，不要更改！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
extern struct Struct_MOTOR_Manage_Object motor_manage_object9;
extern struct Struct_MOTOR_Manage_Object motor_manage_objectA;
extern struct Struct_MOTOR_Manage_Object motor_manage_objectB;

extern void bsp_motor_init(struct Struct_MOTOR_Manage_Object *motor, uint16_t _motor_id);

extern void bsp_motor_state_change(struct Struct_MOTOR_Manage_Object *motor,enum velocity_location_contorl model1,enum pid_control model2,float target);

extern float theta_to_quanshu(uint16_t theta);
extern int32_t quanshu_to_theta(float quanshu);

/* Exported functions -*/



/* Exported variables --------------------------------------------------------*/


#endif
