/*
 * 机械臂再阻尼模式下, 读取关节角度, 并打印末端的位姿
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

void testForwardKinematics(){
    FSARM_JOINTS_STATE_T thetas; // 关节角度
    arm.queryAngle(&thetas); // 查询舵机的角度
    
    FSARM_POINT3D_T toolPosi; // 末端的位置
    float pitch;
    arm.forwardKinematics(thetas, &toolPosi, &pitch); // 正向运动学
    // 打印正向运动学的结果
    DEBUG_SERIAL.println("Tool Posi: X= " + String(toolPosi.x, 1) +\
         ", Y= " + String(toolPosi.y, 1) + \
         ", Z= " + String(toolPosi.z, 1) + \
         ", Pitch: " + String(pitch, 2) + "deg");

}

void setup(){
    DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE); // 初始化串口
    arm.init(); //机械臂初始化
    arm.setDamping(); //设置舵机为阻尼模式
    DEBUG_SERIAL.println("Test Forward Kinematics");
}

void loop(){
    testForwardKinematics();
    delay(1000);
}