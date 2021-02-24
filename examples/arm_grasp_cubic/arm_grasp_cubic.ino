/*
 * 机械臂物块抓取
 * 将物块从一个位置移动到另外一个位置
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2020/05/07
 */
#include "FashionStar_Arm5DoF.h"

#define GRIPPER_OPEN 20.0  // 爪子张开的角度
#define GRIPPER_CLOSE 11.5 // 爪子闭合

FSARM_ARM5DoF arm; //机械臂对象

// 爪子张开
void gripperOpen(){
    arm.setAngle(FSARM_GRIPPER, GRIPPER_OPEN); // 设置爪子的角度,爪子张开
    arm.wait();
}

// 爪子闭合
void gripperClose(){
    arm.setAngle(FSARM_GRIPPER, GRIPPER_CLOSE); // 设置爪子的角度,爪子张开
    arm.wait();
}

// 运行动作组
void run_action_group(){
    // 复位
    arm.move(13.5, 0, 5, 25.0, true);
    // 抬起物块
    gripperOpen();
    arm.move(14.0, 0, -4.0, 55.0, true);
    arm.move(15.6, 0, -7.8, 66.0, true);
    gripperClose();
    arm.move(14.0, 0, -4.0, 55.0, true);
    // 放下物块
    arm.move(15.4, -10.0, -4.0, 55.0, true);
    arm.move(15.4, -10.0, -7.8, 66.0, true);
    gripperOpen();
    arm.move(15.4, -10.0, -4.0, 55.0, true);
    // 复位
    arm.move(13.5, 0, 5, 25.0, true);
}

void setup(){
    arm.init(); //机械臂初始化
    arm.home(); // 机械臂回归到Home的位置
}

void loop(){
    // 运行动作组
    run_action_group();
    // 停顿5s
    delay(5000);
}