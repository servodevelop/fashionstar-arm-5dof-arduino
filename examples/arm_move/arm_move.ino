/*
 * 测试机械臂点控, 从一个点运动到另外一个点
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2021/07/15
 */
#include "FashionStar_Arm5DoF.h"

FSARM_ARM5DoF arm; //机械臂对象

void setup(){
    arm.init(); //机械臂初始化
}

void loop(){
    // 机械臂末端移动到 x1=13.5, y1=0, z1=5 pitch=25°
    // 并且阻塞式等待, 直到机械臂运动到目标角度, 才结束该语句
    arm.move(13.5, 0, 5, 25.0, true);
    delay(1000); // 停顿1s
    
    arm.move(14.0, 0, -4.0, 55.0, true);
    delay(1000); // 停顿1s

    arm.move(9.5, 9.5, -4.0, 55.0, true);
    delay(1000); // 停顿1s
}