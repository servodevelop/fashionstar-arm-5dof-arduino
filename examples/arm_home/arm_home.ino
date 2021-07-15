/*
 * 机械臂末端移动到Home的位置
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2021/07/15
 */

#include "FashionStar_Arm5DoF.h"
FSARM_ARM5DoF arm; //机械臂对象


void setup(){
    arm.init(); //机械臂初始化
    arm.home();
}

void loop(){
}