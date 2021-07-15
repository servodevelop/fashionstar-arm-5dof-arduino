/* 
 * 机械臂阻尼模式与原始角度(Servo Raw Angles)回读
 * 同时也会打印机械臂关节角度(Arm Joint Angles)
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2021/07/15
 */
#include "FashionStar_Arm5DoF.h"

// 调试串口的配置
#if defined(ARDUINO_AVR_UNO)
    #include <SoftwareSerial.h>
    #define SOFT_SERIAL_RX 6
    #define SOFT_SERIAL_TX 7
    SoftwareSerial softSerial(SOFT_SERIAL_RX, SOFT_SERIAL_TX); // 创建软串口
    #define DEBUG_SERIAL softSerial
    #define DEBUG_SERIAL_BAUDRATE 4800
#elif defined(ARDUINO_AVR_MEGA2560)
    #define DEBUG_SERIAL Serial
    #define DEBUG_SERIAL_BAUDRATE 115200
#elif defined(ARDUINO_ARCH_ESP32)
    #define DEBUG_SERIAL Serial
    #define DEBUG_SERIAL_BAUDRATE 115200
#endif 

FSARM_ARM5DoF arm; //机械臂对象

void setup(){
    DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE); // 初始化串口的波特率
    arm.init(); //机械臂初始化
    arm.setDamping(); //设置舵机为阻尼模式
}

void loop(){
    FSARM_JOINTS_STATE_T thetas;    // 关节的原始角度
    float gripper_raw_angle;        // 爪子的原始角度

    arm.queryRawAngle(&thetas);     // 查询并更新关节舵机的原始角度
    gripper_raw_angle = arm.gripper_servo.queryRawAngle(); // 查询爪子的原始角度

    //打印机械臂当前的舵机角度(原始)
    String message = "Servo Raw Angles: [ "+ \
         String(thetas.theta1, 2) + \
         ", " + String(thetas.theta2, 2) + \
         ", " + String(thetas.theta3, 2) + \
         ", " + String(thetas.theta4, 2) + \
         ", " + String(gripper_raw_angle, 2) + " ]";

    DEBUG_SERIAL.println(message);    
    delay(500);

    // 打印机械臂当前的关节角度
    arm.queryAngle(&thetas); //更新舵机角度
    message = "Arm Joint Angles: [ "+ String(thetas.theta1, 2)+\
         ", " + String(thetas.theta2, 2) + \
         ", " + String(thetas.theta3, 2) + \
         ", " + String(thetas.theta4, 2) + " ]";
    
    DEBUG_SERIAL.println(message);    
    delay(500);
}