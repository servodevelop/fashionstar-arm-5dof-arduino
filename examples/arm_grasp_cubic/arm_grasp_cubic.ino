/*
 * 机械臂物块抓取
 * 将物块从一个位置移动到另外一个位置
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2021/07/15
 */
#include "FashionStar_Arm5DoF.h"


FSARM_ARM5DoF arm; //机械臂对象

// 运行动作组
void run_action_group(){
    // 复位
    arm.move(13.5, 0, 5, 25.0, true);
    // 抬起物块
    arm.gripperOpen();
    arm.move(14.0, 0, -4.0, 55.0, true);
    arm.move(15.6, 0, -7.8, 66.0, true);
    arm.gripperClose();
    arm.move(14.0, 0, -4.0, 55.0, true);
    // 放下物块
    arm.move(15.4, -10.0, -4.0, 55.0, true);
    arm.move(15.4, -10.0, -7.8, 66.0, true);
    arm.gripperOpen();
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