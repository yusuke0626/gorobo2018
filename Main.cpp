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
  

  digitalWrite(RunLED, 1);
  //STARTボタンが押されるまで実行
  UPDATELOOP(controller, !(controller.button(START) && controller.button(RIGHT))){ 
  controller2.update();
  if(controller.button(UP)){
    puts("1");
  }
  if(controller2.button(UP)){
    puts("2");
  }
 
    if(controller.button(L1))
      regulation = 0.50;
    else
      regulation = 1;

      
   //全モーターの非常停止。SELECTを押すと作動、もう一度押すと解除
    if(controller.press(SELECT)){
      ms.send(255, 255, 0);
      UPDATELOOP(controller, !controller.press(SELECT));
      cout << "SELECT" << endl;
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

    double lb, lf;
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
    float moter_h = 1;
    float moter_l = 1;

    
    //Right
    right_x = controller.stick(RIGHT_X);

    creg = right_x / 332;
    moter_h = 1 + creg;
    moter_l = 1 - creg;

/*
    if(controller.button(RI) == 1){
      ms.send(6, 2, right_x);
      ms.send(6, 3, ri
      ms.send(6, 2,  left_w * lf * moter_l * regulation);//左前
      ms.send(6, 3,  left_w * lb * moter_l * regulation);//左後
      ms.send(5, 2, -left_w * lb * moter_h * regulation);//右前 
      ms.send(5, 3,  left_w * lf * moter_h * regulation);//右後
      cout << left_w << endl;
*/
  }
  cout << "プログラム終了" << endl;
  digitalWrite(RunLED, 0);
  digitalWrite(19, 0);
  return 0;
}


