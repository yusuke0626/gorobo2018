#include <iostream>
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "RasPiDS3/RasPiDS3.hpp"
#include "RasPiMS/RasPiMS.hpp"
#include <cstdio>

using namespace std;
using namespace RPDS3;
using namespace RPMS;

int main(void){
  
  DualShock3 controller2("/dev/input/js1", false, 0);
  DualShock3 controller("/dev/input/js0", false, 0);
  MotorSerial ms;

  double regulation = 1;  //減速倍率
  double regulation2 = 1;
  bool svh = false;
  bool svl = false;

  int now = 0;
  int pwm = 0;


  int MAX = 250;    //PWMの最大値
  /*-------GPIOピン割り当て-------*/
  //動作確認LED
  int RunLED = 13;
  //x方向リミットスイッチ(ロボット正面の水平方向)
  int x_top = 26;
  int x_bottom = 19;
  //y方向リミットスイッチ(ロボット正面の垂直方向)
  int y_top = 11;
  int y_bottom = 10;
  //z方向リミットスイッチ(高さ方向)
  int z_top = 17;
  int z_bottom = 27;
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
  pinMode(26,INPUT);
  pinMode(19,INPUT);
  pinMode(11,INPUT);
  pinMode(10,INPUT);
  pinMode(17,INPUT);
  pinMode(27,INPUT);
  pinMode(23,OUTPUT);
  pinMode(24,OUTPUT);
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
    if(controller.button(CROSS))
      regulation = 0.50;
    else
      regulation = 1;

    if(controller2.button(L1))
      regulation2 = 0.50;
    else
      regulation2 = 1;

    
    //x軸
    if(controller.button(L1)){
      if(digitalRead(x_top) == true){
        ms.send(7, 4, 0);
      }else{
        ms.send(7, 4, 20 * regulation);
      }
    }else if(controller.button(R1)){
      if(digitalRead(x_bottom) == true){
        ms.send(7, 4, 0);
      }else{
        ms.send(7, 4, -20 * regulation);
      } 
    }else{
      ms.send(7, 4, 0);
    }
   
   //y軸
   if(controller2.button(RIGHT)){
      if(digitalRead(y_top) == true){
        ms.send(7, 2, 0);
      }else{
        ms.send(7, 2, 30 * regulation2);
      }
    }else if(controller2.button(LEFT)){
      if(digitalRead(y_bottom) == true){
       ms.send(7, 2, 0);
      }else{
        ms.send(7, 2, -30 * regulation2);
      }
    }else{
      ms.send(7, 2, 0);
    }

    //z軸
    if(controller2.button(UP)){
      if(digitalRead(z_top) == true){
        ms.send(7, 3, 0);
      }else{
        ms.send(7, 3, 30 * regulation2);
      }
    }else if(controller2.button(DOWN)){
      if(digitalRead(z_bottom) == true){
       ms.send(7, 3, 0);
      }else{
        ms.send(7, 3, -30 * regulation2);
      }
    }else{
      ms.send(7, 3, 0);
    }
    
    //電磁弁つかむとこ
    if(controller2.press(CIRCLE)){
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


    //電磁弁苗木つかむとこ　 
     if(controller2.press(R1)){
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


    
    //苗木の発射
    if(controller2.button(SQUARE))
      ms.send(6, 4, 200);
    else
      ms.send(6, 4, 0);


    if(controller2.button(TRIANGLE))
      ms.send(5, 4, 200);
    else
      ms.send(5, 4, 0);

  
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
    double left_y = 0;
    double left_x = 0;
    double left_theta = 0;
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

    double right_x = 0;
    double creg = 0;

    double right_whr = 1;
    double left_whr = 1;

    
    //Right
    right_x = controller.stick(RIGHT_X);

    creg = fabs(right_x / 256);

    if(right_x > 0){
      left_whr = 1;
      right_whr = creg;
    }else if(right_x < 0){
      left_whr = creg;
      right_whr = 1;
    }else{
      left_whr = 1;
      right_whr = 1;
    }

    int left_t = controller.stick(LEFT_T);
    int right_t = controller.stick(RIGHT_T);
    
    bool la;
    bool ra;
    bool ua;
    bool da;

    if(controller.button(UP)){
       la = 0;
       ra = 0;
       ua = 1;
       da = 0;

       now = now + 1; 

     if(now >= 15){
       pwm = 75;
     }else{
       pwm = now * 5;
     }

      ms.send(6, 2, ua * pwm * regulation);
      ms.send(6, 3, ua * pwm * regulation);
      ms.send(5, 2, ua * -1 * pwm * regulation);
      ms.send(5, 3, ua * -1 * pwm * regulation);
    }else if(controller.button(DOWN)){
      la = 0;
      ra = 0;
      ua = 0;
      da = 1;
      
      now = now + 1; 

      if(now >= 15){
        pwm = 75;
      }else{
       pwm = now * 5;
      }

      ms.send(6, 2, da * -1 * pwm * regulation);
      ms.send(6, 3, da * -1 * pwm * regulation);
      ms.send(5, 2, da * pwm * regulation);
      ms.send(5, 3, da * pwm * regulation);
    }else if(controller.button(RIGHT)){
      la = 0;
      ra = 1;
      ua = 0;
      da = 0;

      now = now + 1; 

      if(now >= 15){
        pwm = 75;
      }else{
        pwm = now * 5;
      }

      ms.send(6, 2, ra * pwm * regulation);
      ms.send(6, 3, ra * -1 * pwm * regulation);
      ms.send(5, 2, ra * pwm * regulation);
      ms.send(5, 3, ra * -1 * pwm * regulation);
    }else if(controller.button(LEFT)){
      la = 1;
      ra = 0;
      ua = 0;
      da = 0;
      
      now = now + 1; 

      if(now >= 15){
        pwm = 75;
      }else{
        pwm = now * 5;
      }

      ms.send(6, 2, la * -1 * pwm * regulation);
      ms.send(6, 3, la * pwm * regulation);
      ms.send(5, 2, la * -1 * pwm * regulation);
      ms.send(5, 3, la * pwm * regulation);

    }else if(controller.stick(RIGHT_T) + 128 > 10){
      ms.send(6, 2, right_t * regulation);
      ms.send(6, 3, right_t * regulation);
      ms.send(5, 2, right_t * regulation);
      ms.send(5, 3, right_t * regulation);
    }else if(controller.stick(LEFT_T) + 128 > 10){
      ms.send(6, 2, -left_t * regulation);
      ms.send(6, 3, -left_t * regulation);
      ms.send(5, 2, -left_t * regulation);
      ms.send(5, 3, -left_t * regulation);
    }else{ 
      pwm = 0;
      now = 0;
      
      ua = 0;
      da = 0;
      la = 0;
      ra = 0;
      
      ms.send(6, 2,  -left_w * lf * regulation * left_whr * 0.5);//左前
      ms.send(6, 3,  -left_w * lb * regulation * left_whr * 0.5);//左後
      ms.send(5, 2,  left_w * lb * regulation * right_whr * 0.5);//右前
      ms.send(5, 3,  left_w * lf * regulation * right_whr * 0.5);//右後
    }

  }
  cout << "プログラム終了" << endl;
  digitalWrite(RunLED, 0);
  digitalWrite(19, 0);
  return 0;
}
