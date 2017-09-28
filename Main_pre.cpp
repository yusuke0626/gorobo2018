#include <iostream>
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include "RasPiDS3/RasPiDS3.hpp"
#include "RasPiMS/RasPiMS.hpp"
#include <cstdio>

using namespace std;
using namespace RPDS3;
using namespace RPMS;

int main(void){   
  DualShock3 controller;
  MotorSerial ms;

  int MAX = 190;    //PWMの最大値
  /*-------GPIOピン割り当て-------*/
  //動作確認LED
  int RunLED = 13;
  //リミットスイッチ
  int lsA = 5;//左上
  int lsB = 6;//左下(反対)
  int lsD = 8;//右下
  int lsE = 9;//右上
  int Air_1 = 17;
  int Air_2 = 19;


  
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
  pinMode(lsA, INPUT);
  pinMode(lsB, INPUT);
  pinMode(lsD, INPUT);
  pinMode(lsE, INPUT);
  pinMode(Air_1, OUTPUT);
  pinMode(Air_2, OUTPUT);
  //リミットスイッチのプルダウン
  pullUpDnControl(lsA, PUD_DOWN);
  pullUpDnControl(lsB, PUD_DOWN);
  pullUpDnControl(lsD, PUD_DOWN);
  pullUpDnControl(lsE, PUD_DOWN);

  digitalWrite(RunLED, 1);
  //STARTボタンが押されるまで実行
  UPDATELOOP(controller, !(controller.button(START) && controller.button(RIGHT))){ 

   //横の開くやつ

    if(controller.button(CIRCLE)){
      if(digitalRead(lsB) == 1){
        ms.send(14, 3, MAX*0.5);
        ms.send(4, 3, MAX*0.5);
      }else{
         ms.send(14, 3, 0);
         ms.send(4, 3, 0);
      }
    }else if(controller.button(CROSS)){
      if(digitalRead(lsA) == 0){
        ms.send(14, 3, -MAX*0.5);
        ms.send(4, 3, -MAX*0.5);
      }else{
        ms.send(14, 3, 0);
        ms.send(4, 3, 0);
      }
    }else{
      ms.send(14, 3, 0);
      ms.send(4, 3, 0);
    }

    if(controller.button(TRIANGLE)){
      if(digitalRead(lsD) == 0){
        ms.send(14, 2, MAX*0.5);
        ms.send(4, 2, MAX*0.5);
      }else{
        ms.send(14, 2, 0);
        ms.send(4, 2, 0);
      }
    }else if(controller.button(SQUARE)){
      if(digitalRead(lsE) == 0){
        ms.send(14, 2, -MAX*0.5);
        ms.send(4, 2, -MAX*0.5);
      }else{
        ms.send(14, 2, 0);
        ms.send(4, 2, 0);
      }
    }else{
      ms.send(14, 2, 0);
      ms.send(4, 2, 0);
    }

   //電磁弁
    static bool Air_one = false;
    static bool Air_two = false;
    if(controller.press(R1)){
      Air_one = !Air_one;
    }
    if(controller.press(L1)){
      Air_two = !Air_two;
    }

    if(Air_one == true){
      digitalWrite(Air_1, 1);
      cout << "Air_1" << endl;
    }else{
      digitalWrite(Air_1, 0);
    }

    if(Air_two == true){
      digitalWrite(Air_2, 1);
      cout << "Air_2" << endl;
    }else{
      digitalWrite(Air_2, 0);
    }

    //スライド

    if(controller.button(L2))
      ms.send(26, 3, MAX);
    else if(controller.button(R2))
      ms.send(26, 3, -MAX);
    else
      ms.send(26, 3, 0);

   
    //全モーターの非常停止。SELECTを押すと作動、もう一度押すと解除
    if(controller.press(SELECT)){
      ms.send(255, 255, 0);
      UPDATELOOP(controller, !controller.press(SELECT));
      cout << "SELECT" << endl;
    }
 
   //--------------ここから足回り(フランジタイヤによる移動)--------------
   //上を押している間のみ、右スティックで前後平行移動 
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
 
    ms.send(17, 3, lf * left_w * 0.70);
    ms.send(18, 2, -(lb * left_w * 0.70));

    //Right
    double right_x = 0;
    double right_y = 0;
    double right_theta = 0;
    int right_w = 0;

    double rf, rb;

    right_x = controller.stick(RIGHT_X);
    right_y = controller.stick(RIGHT_Y);
    right_w = sqrt(pow(right_x, 2) + pow(right_y, 2)) * 2;
    if(right_w > MAX)
      right_w = MAX;
  

    right_theta = (atan2(-right_y, right_x) + M_PI);

    if(right_theta >= 0 && right_theta <= (M_PI/2)){
      rf = -(right_theta * 4 / M_PI) + 1;
      rb = -1;
    }else if(right_theta > (M_PI/2) && right_theta <= (M_PI)){
      rf = -1;
      rb = (right_theta * 4 / M_PI) - 3;
    }else if(right_theta > (M_PI) && right_theta <= (3*M_PI/2)){
      rf = (right_theta * 4 / M_PI) - 5;
      rb = 1;
    }else if(right_theta > (3*M_PI/2) && right_theta <= (2*M_PI)){
      rf = 1;
      rb = -(right_theta * 4 / M_PI) + 7;     
    }

     ms.send(18, 3, rf * right_w * 0.70);
     ms.send(17, 2, -(rb * right_w * 0.70)); 
  }
  cout << "プログラム終了" << endl;
  digitalWrite(RunLED, 0);
  return 0;
}
