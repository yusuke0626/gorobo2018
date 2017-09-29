#include <iostream>
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "RasPiDS3/RasPiDS3.hpp"
#include "RasPiMS/RasPiMS.hpp"
#include <cstdio>

#define x_top 26
#define y_top 11
#define z_top 17
#define x_bottom 19
#define y_bottom 10
#define z_bottom 27

using namespace std;
using namespace RPDS3;
using namespace RPMS;

int main(void){
  
  DualShock3 controller2("/dev/input/js1", false, 0);
  DualShock3 controller("/dev/input/js0", false, 0);
  MotorSerial ms;

  float regulation = 1;  //減速倍率i
  bool svh = false;
  bool svl = false;

  int MAX = 250;    //PWMの最大値
  /*-------GPIOピン割り当て-------*/
  //動作確認LED
  int RunLED = 13;
  /*-------割り当てここまで-------*/
  cout << "Main start." << endl;
 //MDD通信セットアップ
  try{
		ms.init();
	}
	catch(const char *str){
    return -1;
	}


  pinMode(RunLED, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(x_top, INPUT);
  pinMode(y_top, INPUT);
  pinMode(z_top, INPUT);
  pinMode(x_bottom, INPUT);
  pinMode(y_bottom, INPUT);
  pinMode(z_bottom, INPUT);

  digitalWrite(RunLED, 1);
  //STARTボタンが押されるまで実行
  UPDATELOOP(controller, !(controller.button(START) && controller.button(RIGHT))){ 
    controller2.update();

    //レギュレーション 
    if(controller.button(L1))
      regulation = 0.50;
    else
      regulation = 1;

    //刀x軸

    int right_t = controller.stick(RIGHT_T); 
    int left_t = controller.stick(LEFT_T);
    int x_r2 = 0;
    int x_l2 = 0;
    
    x_r2 = right_t + 128;
    x_l2 = left_t + 128;

    //x軸
    if(x_r2 > 0){
      if(digitalRead(x_top) == true){
        ms.send(7, 4, 0);
      }else{
        ms.send(7, 4, x_r2 * 0.2);
      }
    }else if(x_l2 > 0){
      if(digitalRead(x_bottom) == true){
        ms.send(7, 4, 0);
      }else{
        ms.send(7, 4, x_l2 * -0.2);
      } 
    }else{
      ms.send(7, 4, 0);
    }
   
   //y軸
   if(controller2.button(RIGHT)){
      if(digitalRead(y_top) == true){
        ms.send(7, 2, 0);
      }else{
        ms.send(7, 2, 30 * regulation);
      }
    }else if(controller2.button(LEFT)){
      if(digitalRead(y_bottom) == true){
       ms.send(7, 2, 0);
      }else{
        ms.send(7, 2, -30 * regulation);
      }
    }else{
      ms.send(7, 2, 0);
    }

    //z軸
    if(controller2.button(UP)){
      if(digitalRead(z_top) == true){
        ms.send(7, 3, 0);
      }else{
        ms.send(7, 3, 30 * regulation);
      }
    }else if(controller2.button(DOWN)){
      if(digitalRead(z_bottom) == true){
       ms.send(7, 3, 0);
      }else{
        ms.send(7, 3, -30 * regulation);
      }
    }else{
      ms.send(7, 3, 0);
    }
    
    //電磁弁つかむとこ
    if(controller.press(SQUARE)){
      if(svh == true){
        svh = false;
      }else{
        svh = true;
      }
    }

      
    if(svh == true){
      digitalWrite(6, 1);
      digitalWrite(5, 1);
    }else{
      digitalWrite(6, 0);
      digitalWrite(5, 0);
    }

    //電磁弁持ち上げるとこ   
     if(controller.press(CIRCLE)){
      if(svl == true){
        svl = false;
      }else{
        svl = true;
      }
    }
  
    if(svl == true){
      digitalWrite(24, 1);
      digitalWrite(23, 1);
    }else{
      digitalWrite(24, 0);
      digitalWrite(23, 0);
    }

    
    //モーターで解除するやつ
    if(controller.button(TRIANGLE))
      ms.send(6, 4, 200);
     else
      ms.send(6, 4, 0);
  
   //全モーターの非常停止。SELECTを押すと作動、もう一度押すと解除
    if(controller.press(SELECT)){
      ms.send(255, 255, 0);
      UPDATELOOP(controller, !controller.press(SELECT));
      cout << "SELECT" << endl;
    }
    
    if(controller2.press(SELECT)){
      ms.send(255, 255, 0);
      UPDATELOOP(controller2, !controller2.press(SELECT));
      cout << "SELECT2" << endl;
    }
   //--------------ここから足回り(メカナムホイールによる移動)--------------
   // 左スティックによる全方位移動//
    float left_y = 0;
    float left_x = 0;
    float left_theta = 0;
    int left_w = 0;

    //Left
    left_x = controller.stick(LEFT_X);
    left_y = controller.stick(LEFT_Y);
    left_w = sqrt(pow(left_x, 2) + pow(left_y, 2)) * 2;
    if(left_w > MAX)
      left_w = MAX;

    float lb, lf;
    left_theta = (atan2(-left_y, left_x) + M_PI);

    if(left_theta >= 0 && left_theta <= (M_PI/2)){
      lb = (left_theta * 4 / M_PI) - 1;
      lf = 1;
    }else if(left_theta > (M_PI/2) && left_theta <= (M_PI)){
      lb = 1;
      lf = -(left_theta * 4 / M_PI) + 3;
    }else if(left_theta > (M_PI) && left_theta <= (3*M_PI/2)){
      lb = -(left_theta * 4 / M_PI) + 5;
      lf = -1;
    }else if(left_theta > (3*M_PI/2) && left_theta <= (2*M_PI)){
      lb = -1;
      lf = (left_theta * 4 / M_PI) - 7;
    }

    float right_x = 0;
    float creg = 0;

    float right_whr = 1;
    float left_whr = 1;

    
    //Right
    right_x = controller.stick(RIGHT_X);

    creg = fabs(right_x / 256);

    if(right_x > 0){
      left_whr = 1;
      right_whr = creg;
    }else{
      left_whr = creg;
      right_whr = 1;
    }
    

    if(controller.button(UP)){
      ms.send(6, 2, 150);
      ms.send(6, 3, 150);
      ms.send(5, 2, -150);
      ms.send(5, 3, 150);
    }else if(controller.button(DOWN)){
      ms.send(6, 2, -150);
      ms.send(6, 3, -150);
      ms.send(5, 2, 150);
      ms.send(5, 3, -150);
    }else if(controller.button(RIGHT)){
      ms.send(6, 2,  150);
      ms.send(6, 3, -150);
      ms.send(5, 2,  150);
      ms.send(5, 3,  150);
    }else if(controller.button(LEFT)){
      ms.send(6, 2, -150);
      ms.send(6, 3, 150);
      ms.send(5, 2, -150);
      ms.send(5, 3, -150);
    }else{
      ms.send(6, 2,  left_w * lf * regulation * left_whr);//左前
      ms.send(6, 3,  left_w * lb * regulation * left_whr);//左後
      ms.send(5, 2, -left_w * lb * regulation * right_whr);//右前
      ms.send(5, 3,  left_w * lf * regulation * right_whr);//右後
    }

  }
  cout << "プログラム終了" << endl;
  digitalWrite(RunLED, 0);
  digitalWrite(19, 0);
  return 0;
}


