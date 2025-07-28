// #include "key.h"
// #include "car_bsp/car_bsp.h"

// char key_buffer[64];
// uint16_t key_index=0;
// uint16_t count=0;
// bool load_flag=false;
// const struct {
//     GPIO_Regs* gpio;
//     uint32_t pin;
// } SENSOR_PINS[8] = {
//     {GPIOB, KEY_H1_PIN}, 
//     {GPIOB, KEY_H2_PIN},  
//     {GPIOB, KEY_H3_PIN}, 
//     {GPIOA, KEY_H4_PIN},  
//     {GPIOB, KEY_V1_PIN}, 
//     {GPIOB, KEY_V2_PIN},  
//     {GPIOA, KEY_V3_PIN},   
//     {GPIOB, KEY_V4_PIN}  
// };

// int getKeyValue(void)
// {
//     int i, j = 0;
//     int key_value = 0;
//     for (i = 0; i < 4; i++)
//     {
//         delay_us(100);
//         DL_GPIO_setPins(SENSOR_PINS[i].gpio, SENSOR_PINS[i].pin);
//         DL_GPIO_clearPins(SENSOR_PINS[(i+1)%4].gpio, SENSOR_PINS[(i+1)%4].pin);
//         DL_GPIO_clearPins(SENSOR_PINS[(i+2)%4].gpio, SENSOR_PINS[(i+2)%4].pin);
//         DL_GPIO_clearPins(SENSOR_PINS[(i+3)%4].gpio, SENSOR_PINS[(i+3)%4].pin);

//          delay_ms(1);

//         for (j = 0; j < 4; j++)
//         {
//                 if (DL_GPIO_readPins(SENSOR_PINS[j+4].gpio, SENSOR_PINS[j+4].pin) != 0){
//                     key_value = i * 4 + j + 1;
//                 }
//              delay_cycles(1000);
//         }

//          delay_cycles(1000);
//     }

//     return key_value; // 没有按下，返回0
// }
// char get_keychar(uint8_t keyboard){
//     static char input='0';
//     switch (keyboard) {
//             case KEY_0:input='0';break;
//             case KEY_1:input='1';break;
//             case KEY_2:input='2';break;
//             case KEY_3:input='3';break;
//             case KEY_4:input='4';break;
//             case KEY_5:input='5';break;
//             case KEY_6:input='6';break;
//             case KEY_7:input='7';break;
//             case KEY_8:input='8';break;
//             case KEY_9:input='9';break;
//             case KEY_A:input='A';break;
//             case KEY_B:input='B';break;
//             case KEY_C:input='C';break;
//             case KEY_D:input='D';break;
//             case KEY_xin:input='.';break;
//             case KEY_jing:input='#';break;
//             default:break;
//         }
//     if (input=='D') {//delete
//         key_index--;
//     }
//     else if (input=='B') {//go back
//         key_index++;
//     }
//     else if (input=='#') {
//         key_buffer[key_index]='/0';
//         count=key_index;
//         key_index=0;
//         load_flag=true;
//     }
//     else {
//         key_buffer[key_index++]=input;
//     }
//     return input;
// }
// void load_input(void){
//     if (load_flag==true) {
//         if (count<5) {
//             load_flag=false;
//             return;
//         }
//         uint16_t head_index=(key_buffer[0]-'0')*10+key_buffer[1]-'0';
//         //0,1为变量名位  3为正负位（A正C负） 2为是否为浮点数（A不是C是）
//         uint16_t index=4;
//         float content_f=0.0;
//         int content_d=0;
//         char tmp[32];
//         for (int i=0; i<(count-4); i++) {
//             tmp[i]=key_buffer[i+4];
//         }
//         tmp[count-4]='/0';
//         if (key_buffer[2]=='A') {
//             content_d=atoi(tmp);
//             if (key_buffer[3]=='C') {
//                 content_d-=content_d;
//             }
//         }
//         else if (key_buffer[2]=='C') {
//             content_d=atoi(tmp);
//             if (key_buffer[3]=='C') {
//                 content_f-=content_f;
//             }
//         }
//         else {
//             load_flag=false;
//             return;
//         }

//     }
// }