/**
 * @file bsp_motor.c
 * @author xyz
 * @brief motor相关处理
 * @version 1.1
 * @date 2024-09-18
 * @copyright Transistor BUAA
 */

#include "bsp_motor.h"

 struct Struct_MOTOR_Manage_Object motor_manage_object1={0};
 struct Struct_MOTOR_Manage_Object motor_manage_object2={0};
struct Struct_MOTOR_Manage_Object motor_manage_object3={0};
struct Struct_MOTOR_Manage_Object motor_manage_object4={0};
 struct Struct_MOTOR_Manage_Object motor_manage_object5={0};
struct Struct_MOTOR_Manage_Object motor_manage_object6={0};
 struct Struct_MOTOR_Manage_Object motor_manage_object7={0};
 struct Struct_MOTOR_Manage_Object motor_manage_object8={0};


 
 /**
 * @brief motor初始化
 * @param 电机处理结构体
 * @param id(默认电机1对应电调ID1，对应pid结构体1
 */
 void bsp_motor_init(struct Struct_MOTOR_Manage_Object *motor, uint16_t _motor_id)
{
   motor->can_id = _motor_id;
   motor->omega = 0;
   motor->temperaure = 0;
   motor->encoder = 0;
   motor->torque=0;
	
	motor->i_send=0;
	motor->pre_encoder=0;
	motor->now_encoder=0;
	motor->delta_encoder=0;
	motor->total_encoder=0;
	motor->total_round=0;
	motor->motor_cotrol_way=velocity_control;
   switch (_motor_id)
   {
     case (0x0201):
     {
    motor->v_pid_object=pid1;
			 motor->l_pid_object=pid9;
     }
     break;
     case (0x0202):
     {
    motor->v_pid_object=pid2;
			 motor->l_pid_object=pid10;
     }
     break;
     case (0x0203):
     {
    motor->v_pid_object=pid3;
			 motor->l_pid_object=pid11;
     }
     break;
     case (0x0204):
     {
    motor->v_pid_object=pid4;
			 motor->l_pid_object=pid12;
			 
     }
     break;
     case (0x0205):
     {
    motor->v_pid_object=pid5;
			 motor->l_pid_object=pid13;
     }
     break;
     case (0x0206):
     {
    motor->v_pid_object=pid6;
			 motor->l_pid_object=pid14;
     }
     break;
     case (0x0207):
     {
    motor->v_pid_object=pid7;
			 motor->l_pid_object=pid15;
     }
     break;
     case (0x0208):
     {
    motor->v_pid_object=pid8;
			 motor->l_pid_object=pid16;
     }
     break;
     default:
     {
    
     }
     break;
   }
};

float theta_to_quanshu(uint16_t theta)
{
	
	return theta/8192;
}

int32_t quanshu_to_theta(float quanshu)
{
	return (int32_t)(quanshu*8192);
}


/**
 * @brief pid计算后更改发送报文
 * @param *motor 要用pid控制的电机的地址
 * @param model1  位置环/速度环控制 
 * @param model2  增量式/普通式pid控制
 * @param target  目标速度/位置
 */

void bsp_motor_state_change(struct Struct_MOTOR_Manage_Object *motor,enum velocity_location_contorl model1,enum pid_control model2,float target)
{
	
	if(model1==	velocity_control)
	{
		if(model2==pid_control1)
		{		
		
			switch(motor->can_id)
		{
				case(0x201):
				{
					 motor->i_send=(int16_t)(CAN1_0x200_Tx_Data[0]<<8|CAN1_0x200_Tx_Data[1]);
       motor->i_send+=(int16_t)BSP_PID_Model1_Update(&motor->v_pid_object,(float)motor->omega,target);
				CAN1_0x200_Tx_Data[0]=((uint16_t)motor->i_send)>>8;
        CAN1_0x200_Tx_Data[1]=((uint16_t)motor->i_send)&0xff;
				}
				case(0x202):
				{
					 motor->i_send=(int16_t)(CAN1_0x200_Tx_Data[2]<<8|CAN1_0x200_Tx_Data[3]);
       motor->i_send+=(int16_t)BSP_PID_Model1_Update(&motor->v_pid_object,(float)motor->omega,target);
				CAN1_0x200_Tx_Data[2]=((uint16_t)motor->i_send)>>8;
        CAN1_0x200_Tx_Data[3]=((uint16_t)motor->i_send)&0xff;
				}
				case(0x203):
				{
					 motor->i_send=(int16_t)(CAN1_0x200_Tx_Data[4]<<8|CAN1_0x200_Tx_Data[5]);
       motor->i_send+=(int16_t)BSP_PID_Model1_Update(&motor->v_pid_object,(float)motor->omega,target);
				CAN1_0x200_Tx_Data[4]=((uint16_t)motor->i_send)>>8;
        CAN1_0x200_Tx_Data[5]=((uint16_t)motor->i_send)&0xff;
				}
				case(0x204):
				{
					 motor->i_send=(int16_t)(CAN1_0x200_Tx_Data[6]<<8|CAN1_0x200_Tx_Data[7]);
       motor->i_send+=(int16_t)BSP_PID_Model1_Update(&motor->v_pid_object,(float)motor->omega,target);
				CAN1_0x200_Tx_Data[6]=((uint16_t)motor->i_send)>>8;
        CAN1_0x200_Tx_Data[7]=((uint16_t)motor->i_send)&0xff;
				}
				case(0x205):
				{
					 motor->i_send=(int16_t)(CAN1_0x1ff_Tx_Data[0]<<8|CAN1_0x1ff_Tx_Data[1]);
       motor->i_send+=(int16_t)BSP_PID_Model1_Update(&motor->v_pid_object,(float)motor->omega,target);
				CAN1_0x1ff_Tx_Data[0]=((uint16_t)motor->i_send)>>8;
        CAN1_0x1ff_Tx_Data[1]=((uint16_t)motor->i_send)&0xff;
				}
				case(0x206):
				{
					 motor->i_send=(int16_t)(CAN1_0x1ff_Tx_Data[2]<<8|CAN1_0x1ff_Tx_Data[3]);
       motor->i_send+=(int16_t)BSP_PID_Model1_Update(&motor->v_pid_object,(float)motor->omega,target);
				CAN1_0x1ff_Tx_Data[2]=((uint16_t)motor->i_send)>>8;
        CAN1_0x1ff_Tx_Data[3]=((uint16_t)motor->i_send)&0xff;
				}
				case(0x207):
				{
					 motor->i_send=(int16_t)(CAN1_0x1ff_Tx_Data[4]<<8|CAN1_0x1ff_Tx_Data[5]);
       motor->i_send+=(int16_t)BSP_PID_Model1_Update(&motor->v_pid_object,(float)motor->omega,target);
				CAN1_0x1ff_Tx_Data[4]=((uint16_t)motor->i_send)>>8;
        CAN1_0x1ff_Tx_Data[5]=((uint16_t)motor->i_send)&0xff;
				}
				case(0x208):
				{
					 motor->i_send=(int16_t)(CAN1_0x1ff_Tx_Data[6]<<8|CAN1_0x1ff_Tx_Data[7]);
       motor->i_send+=(int16_t)BSP_PID_Model1_Update(&motor->v_pid_object,(float)motor->omega,target);
				CAN1_0x1ff_Tx_Data[6]=((uint16_t)motor->i_send)>>8;
        CAN1_0x1ff_Tx_Data[7]=((uint16_t)motor->i_send)&0xff;
				}
		}
				
				
				
      
		}
		if(model2==pid_control2)
		{
			
			 	switch(motor->can_id)
		{
				case(0x201):
				{
				
       motor->i_send=(int16_t)BSP_PID_Model2_Update(&motor->v_pid_object,(float)motor->omega,target);
				CAN1_0x200_Tx_Data[0]=((uint16_t)motor->i_send)>>8;
        CAN1_0x200_Tx_Data[1]=((uint16_t)motor->i_send)&0xff;
				}
				case(0x202):
				{
					
       motor->i_send=(int16_t)BSP_PID_Model2_Update(&motor->v_pid_object,(float)motor->omega,target);
				CAN1_0x200_Tx_Data[2]=((uint16_t)motor->i_send)>>8;
        CAN1_0x200_Tx_Data[3]=((uint16_t)motor->i_send)&0xff;
				}
				case(0x203):
				{
       motor->i_send=(int16_t)BSP_PID_Model2_Update(&motor->v_pid_object,(float)motor->omega,target);
				CAN1_0x200_Tx_Data[4]=((uint16_t)motor->i_send)>>8;
        CAN1_0x200_Tx_Data[5]=((uint16_t)motor->i_send)&0xff;
				}
				case(0x204):
				{
				
       motor->i_send=(int16_t)BSP_PID_Model2_Update(&motor->v_pid_object,(float)motor->omega,target);
				CAN1_0x200_Tx_Data[6]=((uint16_t)motor->i_send)>>8;
        CAN1_0x200_Tx_Data[7]=((uint16_t)motor->i_send)&0xff;
				}
				case(0x205):
				{
			
       motor->i_send=(int16_t)BSP_PID_Model2_Update(&motor->v_pid_object,(float)motor->omega,target);
				CAN1_0x1ff_Tx_Data[0]=((uint16_t)motor->i_send)>>8;
        CAN1_0x1ff_Tx_Data[1]=((uint16_t)motor->i_send)&0xff;
				}
				case(0x206):
				{
       motor->i_send=(int16_t)BSP_PID_Model2_Update(&motor->v_pid_object,(float)motor->omega,target);
				CAN1_0x1ff_Tx_Data[2]=((uint16_t)motor->i_send)>>8;
        CAN1_0x1ff_Tx_Data[3]=((uint16_t)motor->i_send)&0xff;
				}
				case(0x207):
				{
				
       motor->i_send=(int16_t)BSP_PID_Model2_Update(&motor->v_pid_object,(float)motor->omega,target);
				CAN1_0x1ff_Tx_Data[4]=((uint16_t)motor->i_send)>>8;
        CAN1_0x1ff_Tx_Data[5]=((uint16_t)motor->i_send)&0xff;
				}
				case(0x208):
				{
					
       motor->i_send=(int16_t)BSP_PID_Model2_Update(&motor->v_pid_object,(float)motor->omega,target);
				CAN1_0x1ff_Tx_Data[6]=((uint16_t)motor->i_send)>>8;
        CAN1_0x1ff_Tx_Data[7]=((uint16_t)motor->i_send)&0xff;
				}
		}
	}
	}
	if(model1==	location_control)
	{
		motor->now_encoder=motor->encoder ;
		motor->delta_encoder=(int32_t)(motor->now_encoder-motor->pre_encoder);
		if(motor->delta_encoder<-4096)
		{
			//编码器正向旋转了一圈
			motor->total_round++;
		}
		if(motor->delta_encoder>4096)
		{
			//编码器逆向旋转了一圈
			motor->total_round--;
		}
		motor->total_encoder=(motor->total_round)*8192+(int32_t)motor->now_encoder;
		motor->pre_encoder=motor->now_encoder;
		motor->target_v=(int16_t)BSP_PID_Model2_Update(&motor->l_pid_object,(float)motor->total_encoder,quanshu_to_theta(target));
		switch(motor->can_id)
		{
			case(0x201):
			{
				if(model2==pid_control2)
		{
		motor->i_send=(int16_t)BSP_PID_Model2_Update(&motor->v_pid_object,(float)motor->omega,(float)motor->target_v);
		}
		if(model2==pid_control1)
		{
			motor->i_send=(int16_t)(CAN1_0x200_Tx_Data[0]<<8|CAN1_0x200_Tx_Data[1]);
       motor->i_send+=(int16_t)BSP_PID_Model1_Update(&motor->v_pid_object,(float)motor->omega,(float)motor->target_v);
		}
		CAN1_0x200_Tx_Data[0]=((uint16_t)motor->i_send)>>8;
    CAN1_0x200_Tx_Data[1]=((uint16_t)motor->i_send)&0xff;
			}
			case(0x202):
			{
				if(model2==pid_control2)
		{
		motor->i_send=(int16_t)BSP_PID_Model2_Update(&motor->v_pid_object,(float)motor->omega,(float)motor->target_v);
		}
		if(model2==pid_control1)
		{
			motor->i_send=(int16_t)(CAN1_0x200_Tx_Data[2]<<8|CAN1_0x200_Tx_Data[3]);
       motor->i_send+=(int16_t)BSP_PID_Model1_Update(&motor->v_pid_object,(float)motor->omega,(float)motor->target_v);
		}
		CAN1_0x200_Tx_Data[2]=((uint16_t)motor->i_send)>>8;
    CAN1_0x200_Tx_Data[3]=((uint16_t)motor->i_send)&0xff;
			}
			case(0x203):
			{
				if(model2==pid_control2)
		{
		motor->i_send=(int16_t)BSP_PID_Model2_Update(&motor->v_pid_object,(float)motor->omega,(float)motor->target_v);
		}
		if(model2==pid_control1)
		{
			motor->i_send=(int16_t)(CAN1_0x200_Tx_Data[4]<<8|CAN1_0x200_Tx_Data[5]);
       motor->i_send+=(int16_t)BSP_PID_Model1_Update(&motor->v_pid_object,(float)motor->omega,(float)motor->target_v);
		}
		CAN1_0x200_Tx_Data[4]=((uint16_t)motor->i_send)>>8;
    CAN1_0x200_Tx_Data[5]=((uint16_t)motor->i_send)&0xff;
			}
			case(0x204):
			{
				if(model2==pid_control2)
		{
		motor->i_send=(int16_t)BSP_PID_Model2_Update(&motor->v_pid_object,(float)motor->omega,(float)motor->target_v);
		}
		if(model2==pid_control1)
		{
			motor->i_send=(int16_t)(CAN1_0x200_Tx_Data[6]<<8|CAN1_0x200_Tx_Data[7]);
       motor->i_send+=(int16_t)BSP_PID_Model1_Update(&motor->v_pid_object,(float)motor->omega,(float)motor->target_v);
		}
		CAN1_0x200_Tx_Data[6]=((uint16_t)motor->i_send)>>8;
    CAN1_0x200_Tx_Data[7]=((uint16_t)motor->i_send)&0xff;
			}
			case(0x205):
			{
				if(model2==pid_control2)
		{
		motor->i_send=(int16_t)BSP_PID_Model2_Update(&motor->v_pid_object,(float)motor->omega,(float)motor->target_v);
		}
		if(model2==pid_control1)
		{
			motor->i_send=(int16_t)(CAN1_0x1ff_Tx_Data[0]<<8|CAN1_0x1ff_Tx_Data[1]);
       motor->i_send+=(int16_t)BSP_PID_Model1_Update(&motor->v_pid_object,(float)motor->omega,(float)motor->target_v);
		}
		CAN1_0x1ff_Tx_Data[0]=((uint16_t)motor->i_send)>>8;
    CAN1_0x1ff_Tx_Data[1]=((uint16_t)motor->i_send)&0xff;
			}
			case(0x206):
			{
				if(model2==pid_control2)
		{
		motor->i_send=(int16_t)BSP_PID_Model2_Update(&motor->v_pid_object,(float)motor->omega,(float)motor->target_v);
		}
		if(model2==pid_control1)
		{
			motor->i_send=(int16_t)(CAN1_0x1ff_Tx_Data[2]<<8|CAN1_0x1ff_Tx_Data[3]);
       motor->i_send+=(int16_t)BSP_PID_Model1_Update(&motor->v_pid_object,(float)motor->omega,(float)motor->target_v);
		}
		CAN1_0x1ff_Tx_Data[2]=((uint16_t)motor->i_send)>>8;
    CAN1_0x1ff_Tx_Data[3]=((uint16_t)motor->i_send)&0xff;
			}
			case(0x207):
			{
				if(model2==pid_control2)
		{
		motor->i_send=(int16_t)BSP_PID_Model2_Update(&motor->v_pid_object,(float)motor->omega,(float)motor->target_v);
		}
		if(model2==pid_control1)
		{
			motor->i_send=(int16_t)(CAN1_0x1ff_Tx_Data[4]<<8|CAN1_0x1ff_Tx_Data[5]);
       motor->i_send+=(int16_t)BSP_PID_Model1_Update(&motor->v_pid_object,(float)motor->omega,(float)motor->target_v);
		}
		CAN1_0x1ff_Tx_Data[4]=((uint16_t)motor->i_send)>>8;
    CAN1_0x1ff_Tx_Data[5]=((uint16_t)motor->i_send)&0xff;
			}
			case(0x208):
			{
				if(model2==pid_control2)
		{
		motor->i_send=(int16_t)BSP_PID_Model2_Update(&motor->v_pid_object,(float)motor->omega,(float)motor->target_v);
		}
		if(model2==pid_control1)
		{
			motor->i_send=(int16_t)(CAN1_0x1ff_Tx_Data[6]<<8|CAN1_0x1ff_Tx_Data[7]);
       motor->i_send+=(int16_t)BSP_PID_Model1_Update(&motor->v_pid_object,(float)motor->omega,(float)motor->target_v);
		}
		CAN1_0x1ff_Tx_Data[6]=((uint16_t)motor->i_send)>>8;
    CAN1_0x1ff_Tx_Data[7]=((uint16_t)motor->i_send)&0xff;
			}
			
		}
		
	
	}
};