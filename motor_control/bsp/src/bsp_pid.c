/**
 * @file bsp_pid.c
 * @author xyz
 * @brief pid相关处理
 * @version 1.1
 * @date 2024-09-18
 * @copyright Transistor BUAA
 */

#include "bsp_pid.h"

/**
 * @brief 绝对值限制
 */
static void abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}

struct Struct_PID_Manage_Object pid1={0};
struct Struct_PID_Manage_Object pid2={0};
struct Struct_PID_Manage_Object pid3={0};
struct Struct_PID_Manage_Object pid4={0};
struct Struct_PID_Manage_Object pid5={0};
struct Struct_PID_Manage_Object pid6={0};
struct Struct_PID_Manage_Object pid7={0};
struct Struct_PID_Manage_Object pid8={0};
struct Struct_PID_Manage_Object pid9={0};
struct Struct_PID_Manage_Object pid10={0};
struct Struct_PID_Manage_Object pid11={0};
struct Struct_PID_Manage_Object pid12={0};
struct Struct_PID_Manage_Object pid13={0};
struct Struct_PID_Manage_Object pid14={0};
struct Struct_PID_Manage_Object pid15={0};
struct Struct_PID_Manage_Object pid16={0};

/**
 * @brief pid初始化
 * @param p值
 * @param i值
 * @param d值
 */
void BSP_PID_Init(struct Struct_PID_Manage_Object *pid,float _kp,float _ki,float _kd,float _integral_limit,float _output_limit,float _deadzone)
{
    pid->kp=_kp;
    pid->ki=_ki;
    pid->kd=_kd;

    pid->error=0;
    pid->last_error=0;
	pid->before_last_error=0;
    pid->integral_error=0;

    pid->set_point=0;
    pid->now_point=0;

    pid->p_out=0;
    pid->i_out=0;
    pid->d_out=0;
    pid->output=0;
	
    
    pid->integral_limit=_integral_limit;
    pid->output_limit=_output_limit;
	
	
    pid->deadzone=_deadzone;
	

}

/**
 * @brief 增量式PID控制
 * @param pid处理结构体
 * @param 当前值
 * @param 目标值
 * @return 输出值
 */
float BSP_PID_Model1_Update(struct Struct_PID_Manage_Object *pid,float _now_point,float _set_point)
{
    pid->set_point=_set_point;
    pid->now_point=_now_point;
    pid->error=pid->set_point-pid->now_point;
    pid->p_out=pid->kp*(pid->error-pid->last_error);
    pid->i_out=pid->ki*pid->error;
    pid->d_out=pid->kd*(pid->error-2.0f*pid->last_error+pid->before_last_error);
    abs_limit(&pid->i_out, pid->integral_limit);
    pid->output=pid->p_out+pid->i_out+pid->d_out;
    abs_limit(&pid->output, pid->output_limit);
    pid->before_last_error=pid->last_error;
    pid->last_error=pid->error;
		
    return pid->output;
}
/**
 * @brief 普通式PID控制
 * @param pid处理结构体
 * @param 当前值
 * @param 目标值
 * @return 输出值
 */
float BSP_PID_Model2_Update(struct Struct_PID_Manage_Object *pid,float _now_point,float _set_point)
{
    pid->set_point=_set_point;
    pid->now_point=_now_point;
    pid->error=pid->set_point-pid->now_point;
    pid->p_out=pid->kp*pid->error;
   pid->integral_error+=pid->error;
   abs_limit(&pid->integral_error, pid->integral_limit);
   pid->i_out=pid->ki*pid->integral_error;
   pid->d_out=pid->kd*(pid->error-pid->last_error);
    pid->output=pid->p_out+pid->i_out+pid->d_out;
    abs_limit(&pid->output, pid->output_limit);
	
    pid->last_error=pid->error;

    return pid->output;
}
/**
 * @brief pid参数更改
 * @param p值
 * @param i值
 * @param d值
 */
void pid_change(struct Struct_PID_Manage_Object *pid,float _kp,float _ki,float _kd)
{
	pid->kp=_kp;
	pid->ki=_ki;
	pid->kd=_kd;
}