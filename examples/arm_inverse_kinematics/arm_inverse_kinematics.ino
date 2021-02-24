/*
 * 测试机械臂逆向运动学
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2020/05/07
 */

#include <SoftwareSerial.h>
#include "FashionStar_Arm5DoF.h"
// 软串口的配置
#define SOFT_SERIAL_RX 6
#define SOFT_SERIAL_TX 7
#define SOFT_SERIAL_BAUDRATE 4800
// 创建软串口
SoftwareSerial softSerial(SOFT_SERIAL_RX, SOFT_SERIAL_TX);
FSARM_ARM5DoF arm; //机械臂对象

void setup(){
    softSerial.begin(SOFT_SERIAL_BAUDRATE);
    arm.init(); //机械臂初始化

    // 测试正向运动学
    softSerial.println("Test Forward Kinematics");
    FSARM_JOINTS_STATE_T thetas; // 关节角度
    FSARM_POINT3D_T toolPosi; // 末端位置
    float pitch; // 末端的俯仰角 (deg)
    
    thetas.theta1 = 45.0;
    thetas.theta2 = -130.0;
    thetas.theta3 = 90.0;
    thetas.theta4 = 60.0;
    thetas.gripper = 0.0;
    arm.setAngle(thetas);  // 机械臂运动到目标角度
    arm.wait();

    // 测试逆向运动学
    // 用正向运动学的结果验证逆向运动学的结果
    arm.forwardKinematics(thetas, &toolPosi, &pitch); // 正向运动学
    // 打印正向运动学的结果
    softSerial.println("Tool Posi: X= " + String(toolPosi.x, 1) +\
         ", Y= " + String(toolPosi.y, 1) + \
         ", Z= " + String(toolPosi.z, 1));

    // 逆向运动学
    softSerial.println("Test Inverse Kinematics");
    FSARM_JOINTS_STATE_T thetas_ret; // 关节角度-逆向运动学输出的结果

    softSerial.println("Pitch = "+String(pitch, 2) + " deg");
    FSARM_STATUS code = arm.inverseKinematics(toolPosi, pitch, &thetas_ret);
    softSerial.println("code = "+String(code, DEC));
    softSerial.println("thetas = [" + String(thetas_ret.theta1, 2) + ", "\
        + String(thetas_ret.theta2, 2) + ", "\
        + String(thetas_ret.theta3, 2) + ", "\
        + String(thetas_ret.theta4, 2) + "]");
    
}

void loop(){

}