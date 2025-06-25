// laser_tracker.c（修改同步控制为顺序控制）
#include "laser_tracker.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>

//变量声明
float general_angle=0;

uint16_t servo[4]={center_x1,center_y1,0,0};
uint8_t find_target=0;
uint8_t track_flag=3;
float search_delt=20;//未找到目标每次转动角度
uint16_t lose_time=0;
int laser_x=610;
int laser_y=540;

//函数声明
void laser_track(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4) {

    static float error_x = 0;           // X轴误差
    static float error_y = 0;           // Y轴误差
    static float integral_x = 0;        // X轴积分项
    static float integral_y = 0;        // Y轴积分项
	  static float derivative_x=0;
	  static float derivative_y=0;
    static float prev_error_x = 0;      // 上一次X轴误差
    static float prev_error_y = 0;      // 上一次Y轴误差
	  static float output_x=0;
		static float output_y=0;
    
		 //  应用全局角度偏移（粗略方向）
    float base_target_x = center_x1;
	  float base_target_y = center_y1;
    if (general_angle >= -180.0 && general_angle <= 180.0) {
        base_target_x = center_x1 + general_angle / 360.0 * 4096.0;
    }
		//if(find_target==1)
			//search_delt=1;
		if(find_target==0)
			lose_time++;
		else{
			lose_time=0;
			search_delt=20;
		}
		if(lose_time<100&&track_flag!=0){
			 // 计算四个点的几何中心（重心）
			if(find_target==1){
				float centroid_x = (x1 + x2 + x3 + x4) / 4.0f;
				float centroid_y = (y1 + y2 + y3 + y4) / 4.0f;
				if(centroid_x<0)centroid_x=0;
				if(centroid_y<0)centroid_y=0;
				// 3. 在粗略方向基础上应用几何中心微调（精确锁定）
				float fine_adjust_x = centroid_x - laser_x;  // 几何中心相对于激光位置的偏移
				float fine_adjust_y = centroid_y - laser_y;  // 注意Y轴方向
				if(abs(fine_adjust_x)<((x2+x3)/2-(x1+x4)/2)/6)
					fine_adjust_x=0;
				if(abs(fine_adjust_y)<((y4+y3)/2-(y1+y2)/2)/3||abs(fine_adjust_x)>((x2+x3)/2-(x1+x4)/2)/2)//x方向偏差太多时先不调整y方向
					fine_adjust_y=0;
				// 限制微调范围（避免过大调整）
				if (fine_adjust_x > MAX_FINE_ADJUST) fine_adjust_x = MAX_FINE_ADJUST;
				if (fine_adjust_x < -MAX_FINE_ADJUST) fine_adjust_x = -MAX_FINE_ADJUST;
				if (fine_adjust_y > MAX_FINE_ADJUST) fine_adjust_y = MAX_FINE_ADJUST;
				if (fine_adjust_y < -MAX_FINE_ADJUST) fine_adjust_y = -MAX_FINE_ADJUST;
				// 5. PID控制计算
			  // 比例项（当前误差）
			  error_x = fine_adjust_x;
			  error_y = fine_adjust_y;  // 注意Y轴方向
				// 积分项（带抗饱和）
				integral_x += error_x;
				integral_y += error_y;
				
				// 限制积分项
				integral_x = (integral_x > MAX_INTER_X) ? MAX_INTER_X : (integral_x < -MAX_INTER_X) ? -MAX_INTER_X : integral_x;
				integral_y = (integral_y > MAX_INTER_Y) ? MAX_INTER_Y : (integral_y < -MAX_INTER_Y) ? -MAX_INTER_Y : integral_y;
					
				// 微分项
				float derivative_x = error_x - prev_error_x;
				float derivative_y = error_y - prev_error_y;
				
				// 保存当前误差用于下次计算
				prev_error_x = error_x;
				prev_error_y = error_y;
				// PID输出
				output_x = (KP_x * error_x + KI_x * integral_x + KD_x * derivative_x)*0.9+0.1*output_x;
				output_y = (KP_y * error_y + KI_y * integral_y + KD_y * derivative_y)*0.9+0.1*output_y;	
			}
			else{
				// 积分项（带抗饱和）
				if(error_x<=100)
					error_x=100;
				integral_x += error_x;
				integral_y += error_y;
				
				// 限制积分项
				integral_x = (integral_x > MAX_INTER_X) ? MAX_INTER_X : (integral_x < -MAX_INTER_X) ? -MAX_INTER_X : integral_x;
				integral_y = (integral_y > MAX_INTER_Y) ? MAX_INTER_Y : (integral_y < -MAX_INTER_Y) ? -MAX_INTER_Y : integral_y;
				//output_x = (KP_x * error_x + KI_x * integral_x)*0.9+0.1*output_x;
				//output_y = (KP_y * error_y + KI_y * integral_y)*0.9+0.1*output_y;	
				output_x+=derivative_x*4;
				output_y+=derivative_y*0.1;
			}
			// 6. 更新舵机位置
			float new_x = base_target_x + output_x;
			float new_y = base_target_y + output_y;
			
			if (new_x < MIN_X) {
					servo[0] = MAX_X+1 + new_x;
			} else if (new_x > MAX_X) {
					servo[0] = new_x - MAX_X-1;
			} else {
					servo[0] = new_x;
			}
    
			if(new_y>MAX_Y)
				servo[1]=MAX_Y;
			else if(new_y<MIN_Y)
				servo[1]=MIN_Y;
			else
				servo[1] = new_y;
			 // 9. 计算移动速度（使用PID输出作为速度指示）
			float x_speed,y_speed;
			if(derivative_x<20)
				x_speed=abs(output_x);
			else if(derivative_x<100)
				x_speed=abs(output_x+10*derivative_x);
			else
			  x_speed = abs(output_x+5*derivative_x);
			if(derivative_y<10)
				x_speed=abs(output_x);
			else
			  x_speed = abs(output_x)+2*abs(derivative_x);
			
			if(x_speed>=1500)
				x_speed=1500;
			if(y_speed>=100)
				y_speed=100;
			// 限制最小速度（避免舵机抖动）
			if (x_speed < 5) x_speed = 5;
			if (y_speed < 5) y_speed = 5;
			// 10. 设置舵机位置
			if(track_flag==1||track_flag==3)
				Servo_SetPosition(0x01, servo[0],x_speed);
			if(track_flag==2||track_flag==3)
			  Servo_SetPosition(0x02, servo[1],y_speed);
			find_target=0;
	  }
		else if(lose_time>=100&&track_flag!=0){
			prev_error_x=0;
			prev_error_y=0;
			integral_x=0;
			integral_y=0;
			static uint16_t count=0;
			if(count++>=8){
				if(error_x>=0.0)
					general_angle+=search_delt;
				else
					general_angle-=search_delt;
				count=0;
			}
			if(general_angle>=180||general_angle<=-180){
				search_delt=-search_delt;
				if(general_angle>=180)
					general_angle=179.9;
				else
					general_angle=-179.9;
				count=0;
			}
			if (base_target_x < 0) {
						servo[0] = 4096 + base_target_x;
				} 
			else if (base_target_x > 4095) {
						servo[0] = base_target_x- 4096;
				} 
			else {
						servo[0] = base_target_x;
				}
			
				// Y轴不循环，直接赋值
			servo[1] = base_target_y;
			Servo_SetPosition(1, servo[0], 10000);
			Servo_SetPosition(2, servo[1], 10000);
		}
}
