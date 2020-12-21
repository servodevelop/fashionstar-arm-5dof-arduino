#include "FashionStar_Arm5DoF.h"

FSARM_ARM5DoF::FSARM_ARM5DoF(){

}

// 初始化
void FSARM_ARM5DoF::init(){
    this->protocol.init(115200); // 初始化通信协议
    initServos(); // 初始化所有的舵机
    calibration(); // 机械臂标定
    setAngleRange(); // 设置舵机角度范围
    setSpeed(100); // 初始化舵机的转速
}

// 初始化舵机
void FSARM_ARM5DoF::initServos(){
    for(uint8_t sidx=0; sidx < FSARM_SERVO_NUM; sidx++){
        this->servos[sidx].init(sidx, &this->protocol);
    }
}

// 关节标定
void FSARM_ARM5DoF::calibration(){
    this->servos[FSARM_JOINT1].calibration(FSARM_JOINT1_P90, 90.0, FSARM_JOINT1_N90, -90.0);
    this->servos[FSARM_JOINT2].calibration(FSARM_JOINT2_P0, 0.0, FSARM_JOINT2_N90, -90.0);
    this->servos[FSARM_JOINT3].calibration(FSARM_JOINT3_P90, 90.0, FSARM_JOINT3_N90, -90.0);
    this->servos[FSARM_JOINT4].calibration(FSARM_JOINT4_P90, 90.0, FSARM_JOINT4_N90, -90.0);
    this->servos[FSARM_GRIPPER].calibration(FSARM_GRIPPER_P0, 0.0, FSARM_GRIPPER_P90, 90.0);
}

// 设置舵机的角度范围
void FSARM_ARM5DoF::setAngleRange(){
    this->servos[FSARM_JOINT1].setAngleRange(FSARM_JOINT1_MIN, FSARM_JOINT1_MAX);
    this->servos[FSARM_JOINT2].setAngleRange(FSARM_JOINT2_MIN, FSARM_JOINT2_MAX);
    this->servos[FSARM_JOINT3].setAngleRange(FSARM_JOINT3_MIN, FSARM_JOINT3_MAX);
    this->servos[FSARM_JOINT4].setAngleRange(FSARM_JOINT4_MIN, FSARM_JOINT4_MAX);
    this->servos[FSARM_GRIPPER].setAngleRange(FSARM_GRIPPER_MIN, FSARM_GRIPPER_MAX);
}

// 开启扭矩
void FSARM_ARM5DoF::setTorque(bool enable){
    for(uint8_t sidx=0; sidx < FSARM_SERVO_NUM; sidx++){
        this->servos[sidx].setTorque(enable);
    }
}
// 设置阻尼模式
void FSARM_ARM5DoF::setDamping(){
    for(uint8_t sidx=0; sidx < FSARM_SERVO_NUM; sidx++){
        this->servos[sidx].setDamping(1000);
    }
}

// 设置所有舵机的转速
void FSARM_ARM5DoF::setSpeed(FSUS_SERVO_SPEED_T speed){
    for(uint8_t sidx=0; sidx < FSARM_SERVO_NUM; sidx++){
        if (sidx == FSARM_JOINT4){
            this->servos[sidx].setSpeed(2*speed);
        }else{
            this->servos[sidx].setSpeed(speed);
        }
    }
}

// 读取舵机原始角度
void FSARM_ARM5DoF::queryRawAngle(FSARM_JOINTS_STATE_T* thetas){
    thetas->theta1 = this->servos[FSARM_JOINT1].queryRawAngle();
    thetas->theta2 = this->servos[FSARM_JOINT2].queryRawAngle();
    thetas->theta3 = this->servos[FSARM_JOINT3].queryRawAngle();
    thetas->theta4 = this->servos[FSARM_JOINT4].queryRawAngle();
    thetas->gripper = this->servos[FSARM_GRIPPER].queryRawAngle();

}

// 读取角度
// 查询所有舵机的角度并填充在thetas里面
void FSARM_ARM5DoF::queryAngle(FSARM_JOINTS_STATE_T* thetas){
    thetas->theta1 = this->servos[FSARM_JOINT1].queryAngle();
    thetas->theta2 = this->servos[FSARM_JOINT2].queryAngle();
    thetas->theta3 = this->servos[FSARM_JOINT3].queryAngle();
    thetas->theta4 = this->servos[FSARM_JOINT4].queryAngle();
    thetas->gripper = this->servos[FSARM_GRIPPER].queryAngle();
}

// 设置舵机的原始角度
void FSARM_ARM5DoF::setRawAngle(FSARM_JOINTS_STATE_T thetas){
    this->servos[FSARM_JOINT1].setRawAngle(thetas.theta1);
    this->servos[FSARM_JOINT2].setRawAngle(thetas.theta2);
    this->servos[FSARM_JOINT3].setRawAngle(thetas.theta3);
    this->servos[FSARM_JOINT4].setRawAngle(thetas.theta4);
    this->servos[FSARM_GRIPPER].setRawAngle(thetas.gripper);
}

// 设置舵机的角度
void FSARM_ARM5DoF::setAngle(FSARM_JOINTS_STATE_T thetas){
    this->servos[FSARM_JOINT1].setAngle(thetas.theta1);
    this->servos[FSARM_JOINT2].setAngle(thetas.theta2);
    this->servos[FSARM_JOINT3].setAngle(thetas.theta3);
    this->servos[FSARM_JOINT4].setAngle(thetas.theta4);
    this->servos[FSARM_GRIPPER].setAngle(thetas.gripper);
}

// 设置爪子的角度(关节角度)
void FSARM_ARM5DoF::setAngle(uint8_t id, float theta){
    this->servos[id].setAngle(theta);
}

// 设置舵机的角度
void FSARM_ARM5DoF::setAngle(FSARM_JOINTS_STATE_T thetas, uint16_t interval){
    this->servos[FSARM_JOINT1].setAngle(thetas.theta1, interval);
    this->servos[FSARM_JOINT2].setAngle(thetas.theta2, interval);
    this->servos[FSARM_JOINT3].setAngle(thetas.theta3, interval);
    this->servos[FSARM_JOINT4].setAngle(thetas.theta4, interval);
    this->servos[FSARM_GRIPPER].setAngle(thetas.gripper, interval);
}

// 机械臂正向运动学
void FSARM_ARM5DoF::forwardKinematics(FSARM_JOINTS_STATE_T thetas, FSARM_POINT3D_T* toolPosi, float* pitch){
    FSARM_POINT3D_T wristPosi; //腕关节原点的坐标
    // 求解pitch
    *pitch = thetas.theta2 + thetas.theta3 + thetas.theta4;
    // 角度转弧度
    float theta1 = radians(thetas.theta1);
    float theta2 = radians(thetas.theta2);
    float theta3 = radians(thetas.theta3);
    float theta4 = radians(thetas.theta4);
    // 计算腕关节的坐标
    toolPosi->x = cos(theta1) * (FSARM_LINK2*cos(theta2)+FSARM_LINK3*cos(theta2+theta3) + FSARM_LINK4*cos(theta2+theta3+theta4));
    toolPosi->y = sin(theta1) * (FSARM_LINK2*cos(theta2)+FSARM_LINK3*cos(theta2+theta3) + FSARM_LINK4*cos(theta2+theta3+theta4));
    toolPosi->z = -FSARM_LINK2*sin(theta2)-FSARM_LINK3*sin(theta2+theta3)-FSARM_LINK4*sin(theta2+theta3+theta4);
}

// 机械臂逆向运动学
FSARM_STATUS FSARM_ARM5DoF::inverseKinematics(FSARM_POINT3D_T toolPosi, float pitch, FSARM_JOINTS_STATE_T* thetas){
    // 关节弧度
    float theta1 = 0.0;
    float theta2 = 0.0;
    float theta3 = 0.0;
    float theta4 = 0.0;
    FSARM_POINT3D_T wristPosi; // 腕关节坐标
    
    // 根据工具原点距离机械臂基坐标系的直线距离
    float disO2Tool = sqrt(pow(toolPosi.x,2) + pow(toolPosi.y, 2) + pow(toolPosi.z, 2));
    if (disO2Tool > (FSARM_LINK2+FSARM_LINK3+FSARM_LINK4)){
        return FSARM_STATUS_TOOLPOSI_TOO_FAR;
    }

    // 判断腕关节的原点是否在机械臂坐标系的Z轴上
    if (toolPosi.x == 0 && toolPosi.y == 0){
        // 让theta1保持跟原来相同
        theta1 = radians(this->servos[FSARM_JOINT1].queryRawAngle());
    }else{
        // 求解theta1
        theta1 = atan2(toolPosi.y, toolPosi.x);
        thetas->theta1 = degrees(theta1);
        // 判断theta1是否合法
        if (!servos[FSARM_JOINT1].isAngleLegal(thetas->theta1)){
            return FSARM_STATUS_JOINT1_OUTRANGE;
        }
    }

    // 俯仰角, 角度转弧度
    float pitch_rad = radians(pitch);
    // 计算腕关节的位置
    wristPosi.x = toolPosi.x - FSARM_LINK4*cos(pitch_rad)*cos(theta1);
    wristPosi.y = toolPosi.y - FSARM_LINK4*cos(pitch_rad)*sin(theta1);
    wristPosi.z = toolPosi.z + FSARM_LINK4*sin(pitch_rad);
    
    // 计算theta3
    float b;
    if(cos(theta1) !=0){
        b = wristPosi.x / cos(theta1);
    }else{
        b = wristPosi.y / sin(theta1);
    }
    float cos_theta3 = (pow(wristPosi.z, 2)+pow(b,2) - pow(FSARM_LINK2,2) - pow(FSARM_LINK3, 2))/(2*FSARM_LINK2*FSARM_LINK3);
    float sin_theta3 = sqrt(1 - pow(cos_theta3, 2));
    theta3 = atan2(sin_theta3, cos_theta3);
    thetas->theta3 = degrees(theta3);
    if(!servos[FSARM_JOINT3].isAngleLegal(thetas->theta3)){
        return FSARM_STATUS_JOINT3_OUTRANGE;
    }
    // 计算theta2
    float k1 = FSARM_LINK2 + FSARM_LINK3*cos(theta3);
    float k2 = FSARM_LINK3 * sin(theta3);
    float r = sqrt(pow(k1, 2) + pow(k2, 2));
    theta2 = atan2(-wristPosi.z/r, b/r) - atan2(k2/r, k1/r);
    thetas->theta2 = degrees(theta2);
    if(!servos[FSARM_JOINT2].isAngleLegal(thetas->theta2)){
        return FSARM_STATUS_JOINT2_OUTRANGE;
    }
    // 计算theta4
    theta4 = pitch_rad-(theta2 + theta3);
    thetas->theta4 = degrees(theta4);
    if(!servos[FSARM_JOINT4].isAngleLegal(thetas->theta4)){
        return FSARM_STATUS_JOINT4_OUTRANGE;
    }

    // 成功完成求解
    return FSARM_STATUS_SUCCESS;

}
// 机械臂逆向运动学
FSARM_STATUS FSARM_ARM5DoF::inverseKinematics(FSARM_POINT3D_T toolPosi, FSARM_JOINTS_STATE_T* thetas){
    return inverseKinematics(toolPosi, 0.0, thetas);
}

// 机械臂末端移动, 点对点
FSARM_STATUS FSARM_ARM5DoF::move(FSARM_POINT3D_T toolPosi, float pitch){
    FSARM_JOINTS_STATE_T thetas;
    FSARM_STATUS status = inverseKinematics(toolPosi, pitch,  &thetas); // 逆向运动学
    if(status == FSARM_STATUS_SUCCESS){
        // 设置舵机的角度
        this->servos[FSARM_JOINT1].setAngle(thetas.theta1);
        this->servos[FSARM_JOINT2].setAngle(thetas.theta2);
        this->servos[FSARM_JOINT3].setAngle(thetas.theta3);
        this->servos[FSARM_JOINT4].setAngle(thetas.theta4);
        //注: move函数并不会控制爪子
    }
    return status;
}

FSARM_STATUS FSARM_ARM5DoF::move(float tx, float ty, float tz, float pitch){
    FSARM_POINT3D_T toolPosi;
    toolPosi.x = tx;
    toolPosi.y = ty;
    toolPosi.z = tz;
    return move(toolPosi, pitch);
}

FSARM_STATUS FSARM_ARM5DoF::move(float tx, float ty, float tz, float pitch, bool isWait){
    move(tx, ty, tz, pitch);
    if(isWait){
        wait();
    }
}

// home: 回归机械零点, 初始化机械臂的姿态
void FSARM_ARM5DoF::home(){
    move(FSARM_HOME_X, FSARM_HOME_Y, FSARM_HOME_Z, FSARM_HOME_PITCH, true);
}

// 返回机械臂是否空闲
bool FSARM_ARM5DoF::isIdle(){
    bool is_stop = true;
    for(int sidx=0; sidx<FSARM_SERVO_NUM; sidx++){
        is_stop &= this->servos[sidx].isStop();
    }
    return is_stop;
}

// 等待舵机停止
void FSARM_ARM5DoF::wait(){
    for(int sidx=0; sidx<FSARM_SERVO_NUM; sidx++){
        this->servos[sidx].wait();
    }
}

// 更新末端工具的坐标
void FSARM_ARM5DoF::getToolPose(FSARM_POINT3D_T *toolPosi, float *pitch){
    FSARM_JOINTS_STATE_T thetas;
    queryAngle(&thetas);
    forwardKinematics(thetas, toolPosi, pitch);
}
