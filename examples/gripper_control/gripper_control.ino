/*
 * 机械臂夹爪控制
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2021/07/15
 */
#include "FashionStar_Arm5DoF.h"

FSARM_ARM5DoF arm; //机械臂对象

void setup(){
	arm.init(); // 机械臂初始化
	arm.home(); // 机械臂设置为初始位置
}

void loop(){
	arm.gripperClose(); // 机械臂闭合
	delay(4000); 		// 等待4s
 	arm.gripperOpen(); // 机械臂张开
	delay(4000); 		// 等待4s
}