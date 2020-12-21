/*
 * 设置机械臂关节的角度
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
 * 更新时间: 2020/05/07
 */
#include "FashionStar_Arm5DoF.h"

FSARM_ARM5DoF arm; //机械臂对象

void setup(){
    arm.init(); //机械臂初始化
}

void loop(){
    FSARM_JOINTS_STATE_T thetas; // 关节角度
    thetas.theta1 = 45.0;
    thetas.theta2 = -130.0;
    thetas.theta3 = 90.0;
    thetas.theta4 = 60.0;
    thetas.gripper = 45.0; // 爪子的角度

    // 设置
    arm.setAngle(thetas);  // 设置舵机旋转到特定的角度
    arm.wait();            // 等待舵机旋转到目标位置
    
    delay(1000); // 等待1s

    thetas.theta1 = 90.0;
    thetas.gripper = 0.0; // 爪子的角度
    arm.setAngle(thetas);  // 设置舵机旋转到特定的角度
    arm.wait();            // 等待舵机旋转到目标位置
    delay(1000);
}