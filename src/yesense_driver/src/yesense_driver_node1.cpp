/*
 * @Descripttion: 
 * @version: 
 * @Author: Luohai
 * @Date: 2021-05-10 11:19:06
 * @LastEditors: Luohai
 * @LastEditTime: 2021-07-02 10:41:21
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <serial/serial.h>
#include <string>
bool USE_UTC_TIME = false;
ros::Publisher IMU_pub;
/*
数据类型 数据 ID 长度 内容
IMU 温度 * 0x01 2 DATA1 – DATA2
加速度 0x10 12 DATA1 – DATA12
角速度 0x20 12 DATA1 – DATA12
欧拉角 0x40 12 DATA1 – DATA12
四元数 0x41 16 DATA1 – DATA16
采样时间戳* 0x51 4 DATA1 – DATA4
同步输出时间戳 * 0x52 4 DATA1 – DATA4
*/

//enum c语言枚举数据类型
enum YesenseMsg {
    YESENSE_TEMPERATURE = 0x01,
    YESENSE_ACCEL = 0x10,
    YESENSE_ANGLUAR = 0x20,
    YESENSE_EULER = 0x40,
    YESENSE_QUATERNION = 0x41,
    //采样时间戳* 带星号，默认不输出
    //YESENSE_SAMPLESTAMP = 0x51,
    
};


//#pragma 预处理
#pragma pack(1)

//定义传感器温度
struct yesense_temperature {
    int16_t temp_e2;
};

//定义传感器加速度
struct yesense_accel {
    int32_t ax_e6;
    int32_t ay_e6;
    int32_t az_e6;
};

//定义传感器角速度
struct yesense_angular {
    int32_t wx_e6;
    int32_t wy_e6;
    int32_t wz_e6;
};

//定义欧拉角pitch roll yaw
struct yesense_euler {
    int32_t pitch_e6;
    int32_t roll_e6;
    int32_t yaw_e6;
};

//定义传感器四元数组
struct yesense_quaternion {
    int32_t q_e6[4];
};


#pragma pack()
// 一包数据解析结果
struct yesense_msg {
    //tid帧头？
    unsigned tid;
    struct yesense_temperature *temperature;
    struct yesense_accel *accel;
    struct yesense_angular *angular;
    struct yesense_euler *euler;
    struct yesense_quaternion *quaternion;
    struct yesense_utctime *utc;

};



//用于检查传入数据是否完整
static void yesense_checksum(const unsigned char *buf, size_t sz, unsigned char *ck1, unsigned char *ck2)
{   

    /*

    校验和进行检查，公式来产品说明书
    假设校验范围内有 N 个字节(buffer[N]),计算公式如下:
        CK1 = 0 ; CK2 = 0 ;
        For(i=0;i<N;i++)
        {
            CK1 = CK1 + buffer[i];
            CK2 = CK2 + CK1;
        }
    */
    unsigned char c1 = 0, c2 = 0;
    for (int i = 0; i < sz; i++) {
        c1 += buf[i];
        c2 += c1;
    }
    *ck1 = c1;
    *ck2 = c2;
}

//msg读取数据
static void yesense_msg_parse(const unsigned char *buf, size_t sz, struct yesense_msg *msg)
{
    msg->accel = NULL;
    msg->angular = NULL;
    msg->euler = NULL;
    msg->quaternion = NULL;
    msg->temperature = NULL;
    // 利用静态存储解析数据，所以解析部分并不线程安全，也不需要线程安全
    static struct yesense_temperature y_temperature;
    static struct yesense_accel       y_accel;
    static struct yesense_angular     y_angular;
    static struct yesense_euler       y_euler;
    static struct yesense_quaternion  y_quaternion;
    // ROS_INFO("TID=%u, PAYLOAD=%u", msg->tid, sz);
    int idx = 0;
    while (idx < sz) {
        int remain = sz - idx;
        if (remain < 2) {
            ROS_ERROR("unexpected end of message");
            break;
        }
        int id = buf[idx];
        int len = buf[idx + 1];
        const unsigned char *p = &buf[idx + 2];
        if (remain < (2 + len)) {
            ROS_ERROR("unexpected end of message");
            break;
        }
        switch (id) {
        case YESENSE_TEMPERATURE:
            assert(len == sizeof(y_temperature) && "bad temperature msg size");
            memcpy(&y_temperature, p, len);
            msg->temperature = &y_temperature;
            break;
        case YESENSE_ACCEL:
            assert(len == sizeof(y_accel) && "bad accel msg size");
            memcpy(&y_accel, p, len);
            msg->accel = &y_accel;
            break;
        case YESENSE_ANGLUAR:
            assert(len == sizeof(y_angular) && "bad angular msg size");
            memcpy(&y_angular, p, len);
            msg->angular = &y_angular;
            break;
        case YESENSE_EULER:
            assert(len == sizeof(y_euler) && "bad euler msg size");
            memcpy(&y_euler, p, len);
            msg->euler = &y_euler;
            break;
        case YESENSE_QUATERNION:
            assert(len == sizeof(y_quaternion) && "bad quaternion msg size");
            memcpy(&y_quaternion, p, len);
            msg->quaternion = &y_quaternion;
            break;
    
        /*    
    //对应磁场诡异花，磁场强度
	    case 0x30:
	    case 0x31:
	    break;
        */
        default:
            ROS_INFO("unsupported msg id=0x%02X len=%d", id, len);
            break;
        }
        idx += len + 2;
    }
}
double yaww = 0.0;
size_t yesense_process(const unsigned char *buf, size_t len)
{
    // 小于 7 字节无法解析出完整数据
    if (len < 7) {
        return len;
    }
    // 检查头，失败则从下一字节开始
    if (buf[0] != 0x59 || buf[1] != 0x53) {
        return len - 1;
    }
    // 检查长度
    unsigned mlen = buf[4];
    // 总长度是否完成一个包
    if (len < (7 + mlen)) {
        return len;
    }
    // 计算校验值，从 TID 到 PAYLOAD 结束
    unsigned char c1, c2;
    yesense_checksum(buf + 2, 3 + mlen, &c1, &c2);
    if (c1 != buf[5 + mlen] || c2 != buf[6 + mlen]) {
        ROS_ERROR("checksum failed, expect %02x, %02x actual %02x, %02x", c1, c2, buf[5 + mlen], buf[6 + mlen]);
        return len - (7 + mlen);
    }
    uint16_t tid = buf[2] + (buf[3] << 8);
    struct yesense_msg y_msg = {0};
    y_msg.tid = tid;

    yesense_msg_parse(buf + 5, mlen, &y_msg);
    ros::Time imuT;
    

    imuT = ros::Time::now();
    //TODO:
    sensor_msgs::Imu imumsg;
    imumsg.header.frame_id = "imu";
    imumsg.header.stamp = imuT;
    imumsg.linear_acceleration.x = (double)y_msg.accel->ax_e6*1e-6;
    imumsg.linear_acceleration.y = (double)y_msg.accel->ay_e6*1e-6;
    imumsg.linear_acceleration.z = (double)y_msg.accel->az_e6*1e-6;
    imumsg.angular_velocity.x = (double)y_msg.angular->wx_e6*1e-6*M_PI/180;
    imumsg.angular_velocity.y = (double)y_msg.angular->wy_e6*1e-6*M_PI/180;
    imumsg.angular_velocity.z = (double)y_msg.angular->wz_e6*1e-6*M_PI/180;
    imumsg.orientation.w = (double)y_msg.quaternion->q_e6[0]*1e-6;
    imumsg.orientation.x = (double)y_msg.quaternion->q_e6[1]*1e-6;
    imumsg.orientation.y = (double)y_msg.quaternion->q_e6[2]*1e-6;
    imumsg.orientation.z = (double)y_msg.quaternion->q_e6[3]*1e-6;
    IMU_pub.publish(imumsg);
    yaww += (imumsg.angular_velocity.z - 1.1505821405593758e-05)*0.005;
    std::cout<<"yaw "<<yaww<<std::endl;
    std::cout<<"["<<imumsg.header.stamp<<"] "<< "angle: "<<y_msg.euler->pitch_e6*1e-6<<" "<<y_msg.euler->roll_e6*1e-6<<" "<<y_msg.euler->yaw_e6*1e-6<<std::endl;
    return len - (7 + mlen);

}

int main(int argc, char** argv){
    unsigned char buf[4096];
    size_t len = 0;

    ros::init(argc,argv,"yesense_driver");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    int baud_rate = 460800;
    std::string device = "/dev/ttyUSB0";
    std::string imu_topic = "/imu/data_raw";
    IMU_pub = n.advertise<sensor_msgs::Imu>(imu_topic, 100);
    n_private.param<int>("baud_rate", baud_rate, 460800);
    n_private.param<std::string>("device", device, "/dev/ttyUSB0");
    n_private.param<std::string>("imu_topic", imu_topic, "/imu/data_raw");

    BAUDRATE br = SERIAL_B460800;
    if (serial_num2baudrate(baud_rate, &br) != 0) {
        ROS_ERROR("baudrate '%d' invalid", baud_rate);
        return 1;
    }
    SERIAL fd = serial_open(device.c_str(), br);
    if (fd == INVALID_SERIAL) {
        ROS_ERROR("cannot open device=%s", device.c_str());
        return 1;
    }


    while(ros::ok())
    {
        int rv = serial_read(fd, buf + len, sizeof(buf) - len);
        if (rv == -1) {
            ROS_ERROR("read from dev error");
            break;
        }
        // if rv == 0, continue parsing
        len += rv;
        //buf[len == sizeof(buf) ? len - 1 : len] = 0; // 截断，保证 ascii 解析
        size_t newlen = yesense_process(buf, len);
        if (newlen != len) {
            memmove(buf, buf + len - newlen, newlen);
            len = newlen;
        }
        ros::spinOnce();
    }
}
