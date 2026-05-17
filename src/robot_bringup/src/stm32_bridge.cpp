#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <serial/serial.h>
#include <vector>

// 消除字节对齐
// 下发速度包
struct __attribute__((packed)) TxPacket {
    uint8_t header1; // 0xAA
    uint8_t header2; // 0x55
    uint8_t type;    // 0x01
    int16_t vx;      // mm/s
    int16_t vy;      // mm/s
    int16_t vth;     // mrad/s
    uint8_t tail;    // 0xEE
};

// 轮式里程计上传包 (V3，删除 IMU)
struct __attribute__((packed)) RxPacket {
    uint8_t header1; // 0xAA
    uint8_t header2; // 0x55
    uint8_t type;    // 0x02
    int16_t vx;      // mm/s
    int16_t vy;
    int16_t vth;
    uint8_t tail;    // 0xEE
};

serial::Serial ser;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if (!ser.isOpen()) return;

    TxPacket tx;
    tx.header1 = 0xAA;
    tx.header2 = 0x55;
    tx.type = 0x01;
    // float m/s -> int16 mm/s
    tx.vx  = msg->linear.x * 1000;
    tx.vy  = msg->linear.y * 1000;
    tx.vth = msg->angular.z * 1000;
    tx.tail = 0xEE;

    ser.write((const uint8_t*)&tx, sizeof(TxPacket));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "stm32_bridge");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string port_name;
    int baud_rate;
    bool publish_tf;
    nh_private.param<std::string>("port_name", port_name, "/dev/ttyUSB0");
    nh_private.param<int>("baud_rate", baud_rate, 115200);
    
    // V4架构下，Cartographer 负责统一建图与定位，并发布 odom->base_link 的TF，
    // 所以这里 TF 默认必须为 false，避免与 Cartographer 冲突。
    nh_private.param<bool>("publish_tf", publish_tf, false);

    ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 50, cmdVelCallback);
    // V4 ：仅发布 odom_raw 供 Cartographer 的后端优化使用，不再发布 IMU 数据
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom_raw", 50);
    
    //V4
    //tf::TransformBroadcaster odom_broadcaster;

    try {
        ser.setPort(port_name);
        ser.setBaudrate(baud_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (const serial::IOException& e) {
        ROS_ERROR("UART FAILED: %s", e.what());
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO("UART ON, Port: %s, Baudrate: %d", port_name.c_str(), baud_rate);
    }

    double x_pos = 0.0, y_pos = 0.0, th_pos = 0.0;
    ros::Time last_time = ros::Time::now();

    ros::Rate rate(50); // 50Hz频率
    std::vector<uint8_t> rx_buffer;

    while (ros::ok()) {
        if (ser.available()) {
            std::string raw = ser.read(ser.available());
            rx_buffer.insert(rx_buffer.end(), raw.begin(), raw.end());
        }

        while (rx_buffer.size() >= sizeof(RxPacket)) {
            if (rx_buffer[0] == 0xAA && rx_buffer[1] == 0x55 && rx_buffer[2] == 0x02) {
                RxPacket* rxPack = (RxPacket*)rx_buffer.data();
                
                if (rxPack->tail == 0xEE) {
                    // 解析：int16 mm/s -> float m/s
                    double vx = rxPack->vx / 1000.0;
                    double vy = rxPack->vy / 1000.0;
                    double vth= rxPack->vth / 1000.0;

                    ros::Time current_time = ros::Time::now();
                    double dt = (current_time - last_time).toSec();

                    // 航迹推算
                    double dx = (vx * cos(th_pos) - vy * sin(th_pos)) * dt;
                    double dy = (vx * sin(th_pos) + vy * cos(th_pos)) * dt;
                    double dth = vth * dt;

                    x_pos += dx; y_pos += dy; th_pos += dth;
                    last_time = current_time;

                    // Odom 发布
                    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_pos);
                    nav_msgs::Odometry odom;
                    odom.header.stamp = current_time;
                    odom.header.frame_id = "odom";
                    odom.child_frame_id = "base_link";
                    odom.pose.pose.position.x = x_pos;
                    odom.pose.pose.position.y = y_pos;
                    odom.pose.pose.orientation = odom_quat;
                    odom.twist.twist.linear.x = vx;
                    odom.twist.twist.linear.y = vy;
                    odom.twist.twist.angular.z = vth;
                    odom_pub.publish(odom);

                    // // TF 发布 (默认关闭并交由 EKF 节点发布)
                    // if (publish_tf) {
                    //     geometry_msgs::TransformStamped odom_trans;
                    //     odom_trans.header.stamp = current_time;
                    //     odom_trans.header.frame_id = "odom";
                    //     odom_trans.child_frame_id = "base_link";
                    //     odom_trans.transform.translation.x = x_pos;
                    //     odom_trans.transform.translation.y = y_pos;
                    //     odom_trans.transform.rotation = odom_quat;
                    //     odom_broadcaster.sendTransform(odom_trans);
                    // }

                    rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + sizeof(RxPacket));
                } else {
                    rx_buffer.erase(rx_buffer.begin());
                }
            } else {
                rx_buffer.erase(rx_buffer.begin());
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    if (ser.isOpen()) ser.close();
    return 0;
}
