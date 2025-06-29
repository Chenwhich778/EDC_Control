// laser_tracker.cï¿½ï¿½ï¿½Þ¸ï¿½Í¬ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ÎªË³ï¿½ï¿½ï¿½ï¿½Æ£ï¿?
#include "laser_tracker.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>

//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
float general_angle=0;

uint16_t servo[4]={center_x1,center_y1,0,0};
uint8_t find_target=0;
uint8_t track_flag=1;
float search_delt=10;//Î´ï¿½Òµï¿½Ä¿ï¿½ï¿½Ã¿ï¿½ï¿½×ªï¿½ï¿½ï¿½Ç¶ï¿½
uint16_t lose_time=0;
int laser_x=620;
int laser_y=540;
uint16_t buzzer_time=0;
extern float X_enemy;
extern float Y_enemy;
//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
void laser_track(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4) {

    static float error_x = 0;           // Xï¿½ï¿½ï¿½ï¿½ï¿?
    static float error_y = 0;           // Yï¿½ï¿½ï¿½ï¿½ï¿?
    static float integral_x = 0;        // Xï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
    static float integral_y = 0;        // Yï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
	  static float derivative_x=0;
	  static float derivative_y=0;
    static float prev_error_x = 0;      // ï¿½ï¿½Ò»ï¿½ï¿½Xï¿½ï¿½ï¿½ï¿½ï¿?
    static float prev_error_y = 0;      // ï¿½ï¿½Ò»ï¿½ï¿½Yï¿½ï¿½ï¿½ï¿½ï¿?
	  static float output_x=0;
		static float output_y=0;
    
		 //  Ó¦ï¿½ï¿½È«ï¿½Ö½Ç¶ï¿½Æ«ï¿½Æ£ï¿½ï¿½ï¿½ï¿½Ô·ï¿½ï¿½ï¿½
    float base_target_x = center_x1;
	  float base_target_y = center_y1;
    if (general_angle >= -180.0 && general_angle <= 180.0&&abs(general_angle)>=5) {
        base_target_x = center_x1 + general_angle / 360.0 * 4096.0;
    }
		//if(find_target==1)
			//search_delt=1;
		if(find_target==0)
			{lose_time++;
				if(Y_enemy!=0)
				  general_angle=atan2(X_enemy,Y_enemy) * (180.0/ PI);
				prev_error_x=0;
				prev_error_y=0;
				//integral_x=0;
				//integral_y=0;
				//error_x = 0;           // Xï¿½ï¿½ï¿½ï¿½ï¿?
        //error_y = 0;
			}

		else{
			lose_time=0;
			search_delt=10;
				 // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
		}
		if(track_flag!=0){
			 // ï¿½ï¿½ï¿½ï¿½ï¿½Ä¸ï¿½ï¿½ï¿½Ä¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä£ï¿½ï¿½ï¿½ï¿½Ä£ï¿?
			if(find_target==1){
				float centroid_x = (x1 + x2 + x3 + x4) / 4.0f;
				float centroid_y = (y1 + y2 + y3 + y4) / 4.0f;
				if(centroid_x<0)centroid_x=0;
				if(centroid_y<0)centroid_y=0;
				// 3. ï¿½Ú´ï¿½ï¿½Ô·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ó¦ï¿½Ã¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Î?ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½È·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
				float fine_adjust_x = centroid_x - laser_x;  // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ú¼ï¿½ï¿½ï¿½Î»ï¿½Ãµï¿½Æ?ï¿½ï¿½
				float fine_adjust_y = centroid_y - laser_y;  // ×¢ï¿½ï¿½Yï¿½á·½ï¿½ï¿½
				if(abs(fine_adjust_x)<((x2+x3)/2-(x1+x4)/2)*0.8)
					fine_adjust_x=0;
				if(abs(fine_adjust_y)<((y4+y3)/2-(y1+y2)/2)/2||abs(fine_adjust_x)>((x2+x3)/2-(x1+x4)/2)/2)//xï¿½ï¿½ï¿½ï¿½Æ«ï¿½ï¿½Ì«ï¿½ï¿½Ê±ï¿½È²ï¿½ï¿½ï¿½ï¿½ï¿½yï¿½ï¿½ï¿½ï¿½
					fine_adjust_y=0;
				// ÏÞÖÆÎ¢µ÷·¶Î§£¨±ÜÃâ¹ý´óµ÷Õû£©
				if (fine_adjust_x > MAX_FINE_ADJUST) fine_adjust_x = MAX_FINE_ADJUST;
				if (fine_adjust_x < -MAX_FINE_ADJUST) fine_adjust_x = -MAX_FINE_ADJUST;
				if (fine_adjust_y > MAX_FINE_ADJUST) fine_adjust_y = MAX_FINE_ADJUST;
				if (fine_adjust_y < -MAX_FINE_ADJUST) fine_adjust_y = -MAX_FINE_ADJUST;
				// 5. PID¿ØÖÆ¼ÆËã
			  // ±ÈÀýÏî£¨µ±Ç°Îó²î£©
			  error_x = fine_adjust_x;
			  error_y = fine_adjust_y;  // ×¢ÒâYÖá·½Ïò
				// »ý·ÖÏî£¨´ø¿¹±¥ºÍ£©
				integral_x += error_x;
				integral_y += error_y;
				
				// ÏÞÖÆ»ý·ÖÏî
				integral_x = (integral_x > MAX_INTER_X) ? MAX_INTER_X : (integral_x < -MAX_INTER_X) ? -MAX_INTER_X : integral_x;
				integral_y = (integral_y > MAX_INTER_Y) ? MAX_INTER_Y : (integral_y < -MAX_INTER_Y) ? -MAX_INTER_Y : integral_y;
					
				// Î¢·ÖÏî
				float derivative_x = error_x - prev_error_x;
				float derivative_y = error_y - prev_error_y;
				
				// ±£´æµ±Ç°Îó²îÓÃÓÚÏÂ´Î¼ÆËã
				prev_error_x = error_x;
				prev_error_y = error_y;
				// PIDÊä³ö
				output_x = (KP_x * error_x + KI_x * integral_x + KD_x * derivative_x)*0.9+0.1*output_x;
				output_y = (KP_y * error_y + KI_y * integral_y + KD_y * derivative_y)*0.9+0.1*output_y;	
			}
			else{
			prev_error_x=0;
			prev_error_y=0;
			integral_x=0;
			integral_y=0;
			static uint16_t count=0;
			static float add_angle=0;
			if(count++>=8){
				if(error_x>=0.0)
					add_angle+=search_delt;
				else
					add_angle+=search_delt;
				count=0;
			}
			if(add_angle>=10||add_angle<=-10){
				search_delt=-search_delt;
				if(add_angle>=10)
					add_angle=9.9;
				else
					add_angle=-9.9;
				count=0;
				output_x=0;
				output_y=0;
			}
			}
			// 6. ¸üÐÂ¶æ»úÎ»ÖÃ
			float new_x = base_target_x + output_x;
			float new_y = base_target_y + output_y;
			
			servo[0]=(uint16_t)new_x % 4096;
    
			if(new_y>MAX_Y)
				servo[1]=MAX_Y;
			else if(new_y<MIN_Y)
				servo[1]=MIN_Y;
			else
				servo[1] = new_y;
			 // 9. ¼ÆËãÒÆ¶¯ËÙ¶È£¨Ê¹ÓÃPIDÊä³ö×÷ÎªËÙ¶ÈÖ¸Ê¾£©
			float x_speed,y_speed;
			if(derivative_x<20)
				x_speed=abs(output_x);
			else if(derivative_x<100)
				x_speed=abs(output_x+1*derivative_x);
			else
			  x_speed = abs(output_x+0.55*derivative_x);
			
			if(derivative_y<20)
				y_speed=abs(output_y);
			else if(derivative_y<100)
				y_speed=abs(output_y+1*derivative_y);
			else
			  y_speed = abs(output_y+0.5*derivative_y);
			if(x_speed>=1500)
				x_speed=1500;
			if(y_speed>=1500)
				y_speed=1500;
			
//			if(y_speed>=100)
//				y_speed=100;
			// ÏÞÖÆ×îÐ¡ËÙ¶È£¨±ÜÃâ¶æ»ú¶¶¶¯£©
			if (x_speed < 5) x_speed = 5;
			if (y_speed < 5) y_speed = 5;
			// 10. ÉèÖÃ¶æ»úÎ»ÖÃ
			if(find_target==0)
				x_speed=3000;
			if(track_flag==1||track_flag==3)
				Servo_SetPosition(0x01, servo[0],x_speed*0.8);
			if(track_flag==2||track_flag==3)
			  Servo_SetPosition(0x02, servo[1],y_speed*0.8);
			find_target=0;
	  }
//		else if(lose_time>=100&&track_flag!=0){
//			prev_error_x=0;
//			prev_error_y=0;
//			integral_x=0;
//			integral_y=0;
//			static uint16_t count=0;
//			if(count++>=8){
//				if(error_x>=0.0)
//					general_angle+=search_delt;
//				else
//					general_angle-=search_delt;
//				count=0;
//			}
////			if(general_angle>=180||general_angle<=-180){
////				search_delt=-search_delt;
////				if(general_angle>=180)
////					general_angle=179.9;
////				else
////					general_angle=-179.9;
////				count=0;
////			}
//			if(general_angle>=35||general_angle<=-35){
//				search_delt=-search_delt;
//				if(general_angle>=35)
//					general_angle=34.9;
//				else
//					general_angle=-34.9;
//				count=0;
//			}
//			if (base_target_x < 0) {
//						servo[0] = 4096 + base_target_x;
//				} 
//			else if (base_target_x > 4095) {
//						servo[0] = base_target_x- 4096;
//				} 
//			else {
//						servo[0] = base_target_x;
//				}
//			
//				// YÖá²»Ñ­»·£¬Ö±½Ó¸³Öµ
//			servo[1] = base_target_y;
//			Servo_SetPosition(1, servo[0], 2000);
//			Servo_SetPosition(2, servo[1], 2000);
//		}
}
