/**
 * @file bsp_pid.c
 * @author xyz
 * @brief pid��ش���
 * @version 1.1
 * @date 2024-09-18
 * @copyright Transistor BUAA
 */

#include "bsp_pid.h"

/**
 * @brief ����ֵ����
 */
static void abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}


 struct Struct_PID_Manage_Object pid1_1={0};
 struct Struct_PID_Manage_Object pid2_1={0};
 struct Struct_PID_Manage_Object pid3_1={0};
 struct Struct_PID_Manage_Object pid4_1={0};
struct Struct_PID_Manage_Object pid5_1={0};
struct Struct_PID_Manage_Object pid6_1={0};
 struct Struct_PID_Manage_Object pid7_1={0};
 struct Struct_PID_Manage_Object pid8_1={0};
struct Struct_PID_Manage_Object pid9_1={0};
 struct Struct_PID_Manage_Object pidA_1={0};
 struct Struct_PID_Manage_Object pidB_1={0};

struct Struct_PID_Manage_Object pid1_2={0};
struct Struct_PID_Manage_Object pid2_2={0};
 struct Struct_PID_Manage_Object pid3_2={0};
 struct Struct_PID_Manage_Object pid4_2={0};
 struct Struct_PID_Manage_Object pid5_2={0};
 struct Struct_PID_Manage_Object pid6_2={0};
 struct Struct_PID_Manage_Object pid7_2={0};
 struct Struct_PID_Manage_Object pid8_2={0};
 struct Struct_PID_Manage_Object pid9_2={0};
 struct Struct_PID_Manage_Object pidA_2={0};
struct Struct_PID_Manage_Object pidB_2={0};
/**
 * @brief pid��ʼ��
 * @param pֵ
 * @param iֵ
 * @param dֵ
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
 * @brief ����ʽPID����
 * @param pid����ṹ��
 * @param ��ǰֵ
 * @param Ŀ��ֵ
 * @return ���ֵ
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
 * @brief ��ͨʽPID����
 * @param pid����ṹ��
 * @param ��ǰֵ
 * @param Ŀ��ֵ
 * @return ���ֵ
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
 * @brief pid��������
 * @param pֵ
 * @param iֵ
 * @param dֵ
 */
void pid_change(struct Struct_PID_Manage_Object *pid,float _kp,float _ki,float _kd)
{
	pid->kp=_kp;
	pid->ki=_ki;
	pid->kd=_kd;
	}
