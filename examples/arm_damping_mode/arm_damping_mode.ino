/* 
 * 机械臂阻尼模式与原始角度(Servo Raw Angles)回读
 * 同时也会打印机械臂关节角度(Arm Joint Angles)
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
 * 更新时间: 2020/05/07
 */
#include <SoftwareSerial.h>
#include "FashionStar_Arm5DoF.h"

// 软串口的配置
#define SOFT_SERIAL_RX 6
#define SOFT_SERIAL_TX 7
#define SOFT_SERIAL_BAUDRATE 4800

SoftwareSerial softSerial(SOFT_SERIAL_RX, SOFT_SERIAL_TX); // 创建软串口
FSARM_ARM5DoF arm; //机械臂对象

void setup(){
    softSerial.begin(SOFT_SERIAL_BAUDRATE); // 初始化软串口的波特率
    arm.init(); //机械臂初始化
    arm.setDamping(); //设置舵机为阻尼模式
}

void loop(){
    FSARM_JOINTS_STATE_T thetas;
    arm.queryRawAngle(&thetas); //更新舵机角度
    //打印机械臂当前的舵机角度(原始)
    String message = "Servo Raw Angles: [ "+ String(thetas.theta1, 2)+\
         ", " + String(thetas.theta2, 2) + \
         ", " + String(thetas.theta3, 2) + \
         ", " + String(thetas.theta4, 2) + \
         ", " + String(thetas.gripper, 2) + " ]";

    softSerial.println(message);    
    delay(500);

    // 打印机械臂当前的关节角度
    arm.queryAngle(&thetas); //更新舵机角度
    message = "Arm Joint Angles: [ "+ String(thetas.theta1, 2)+\
         ", " + String(thetas.theta2, 2) + \
         ", " + String(thetas.theta3, 2) + \
         ", " + String(thetas.theta4, 2) + \
         ", " + String(thetas.gripper, 2) + " ]";
    
    softSerial.println(message);    
    delay(500);
}