/*
 * 测试机械臂逆向运动学
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
    DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE);
    arm.init(); //机械臂初始化

    // 测试正向运动学
    DEBUG_SERIAL.println("Test Forward Kinematics");
    FSARM_JOINTS_STATE_T thetas; // 关节角度
    FSARM_POINT3D_T toolPosi; // 末端位置
    float pitch; // 末端的俯仰角 (deg)
    
    thetas.theta1 = 45.0;
    thetas.theta2 = -130.0;
    thetas.theta3 = 90.0;
    thetas.theta4 = 60.0;
    arm.setAngle(thetas);  // 机械臂运动到目标角度
    arm.wait();

    // 测试逆向运动学
    // 用正向运动学的结果验证逆向运动学的结果
    arm.forwardKinematics(thetas, &toolPosi, &pitch); // 正向运动学
    // 打印正向运动学的结果
    DEBUG_SERIAL.println("Tool Posi: X= " + String(toolPosi.x, 1) +\
         ", Y= " + String(toolPosi.y, 1) + \
         ", Z= " + String(toolPosi.z, 1));

    // 逆向运动学
    DEBUG_SERIAL.println("Test Inverse Kinematics");
    FSARM_JOINTS_STATE_T thetas_ret; // 关节角度-逆向运动学输出的结果

    DEBUG_SERIAL.println("Pitch = "+String(pitch, 2) + " deg");
    FSARM_STATUS code = arm.inverseKinematics(toolPosi, pitch, &thetas_ret);
    DEBUG_SERIAL.println("code = "+String(code, DEC));
    DEBUG_SERIAL.println("thetas = [" + String(thetas_ret.theta1, 2) + ", "\
        + String(thetas_ret.theta2, 2) + ", "\
        + String(thetas_ret.theta3, 2) + ", "\
        + String(thetas_ret.theta4, 2) + "]");
    
}

void loop(){

}