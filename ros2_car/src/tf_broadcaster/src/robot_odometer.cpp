#include <rclcpp/rclcpp.hpp>                       // ROS2 C++客户端文件
#include <nav_msgs/msg/odometry.hpp>               // ROS2标准消息包 包含里程计/路径/地图等
#include <geometry_msgs/msg/transform_stamped.hpp> // ROS2标准消息包 表示各种几何信息 是所有运动与位姿相关消息的基础。

/* TF 坐标变换系统 的核心头文件 用于在不同坐标系之间广播和转换坐 */
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <fcntl.h>   // 文件控制（file control） open()等
#include <termios.h> // 串口属性控制
#include <unistd.h>  // linux系统调用 read() write()等

#include <string>    // c++字符串类
#include <algorithm> // 通用算法库

class WheelOdomPublisher : public rclcpp::Node
{
public:
    WheelOdomPublisher()
        : Node("odom_publisher")
    {
        // 发布 Odometry
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // TF 广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 打开串口
        serial_fd_ = open("/dev/ttyS5", O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开串口 /dev/ttyS5");
            rclcpp::shutdown();
            return;
        }

        configureSerial(serial_fd_);
        RCLCPP_INFO(this->get_logger(), "串口 /dev/ttyS5 打开成功");

        // 创建定时器 参数1：定时时间 参数2：回调函数
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&WheelOdomPublisher::readSerial, this));
    }

private:
    int serial_fd_;                                                  // 串口文件描述符
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_; // 消息类型为 nav_msgs::msg::Odometry 的发布器只能指针
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  // 创建一个TF广播器的共享智能指针
    rclcpp::TimerBase::SharedPtr timer_;                             // 定时器智能指针

    double last_time_ = 0.0;
    float last_x_ = 0.0f, last_y_ = 0.0f, last_theta_ = 0.0f;

    // 配置串口参数
    void configureSerial(int fd)
    {
        struct termios tty;  // 串口/终端控制结构体
        tcgetattr(fd, &tty); // 获取当前配置

        cfsetospeed(&tty, B115200); // 设置串口输出波特率
        cfsetispeed(&tty, B115200); // 设置串口输入波特率

        tty.c_cflag |= (CLOCAL | CREAD); // 本地连接，允许接收
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;      // 8 位数据位
        tty.c_cflag &= ~PARENB;  // 无校验
        tty.c_cflag &= ~CSTOPB;  // 1 个停止位
        tty.c_cflag &= ~CRTSCTS; // 无硬件流控

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 非规范模式
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // 禁用软件流控
        tty.c_oflag &= ~OPOST;                          // 原始输出模式

        tty.c_cc[VMIN] = 0;  // 最少接受字节 0代表非阻塞
        tty.c_cc[VTIME] = 0; // 等待时间

        tcsetattr(fd, TCSANOW, &tty); // 设置串口参数
    }

    void readSerial()
    {
        char buf[128];

        /* 读取串口数据 没有数据则直接返回 */
        int n = read(serial_fd_, buf, sizeof(buf) - 1);
        if (n <= 0)
            return; // 没有数据直接返回

        /* 构造String类型使用 */
        buf[n] = '\0';

        // 将C字符串数据构造为string类型
        std::string line(buf);

        // 打印接收到的原始串口数据
        // RCLCPP_INFO(this->get_logger(), "串口接收: '%s'", line.c_str());

        // 去掉换行符
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
        line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
        if (line.empty())
            return;

        // 解析 x y theta（空格或逗号分隔）
        std::replace(line.begin(), line.end(), ',', ' ');
        float x, y, theta;
        if (sscanf(line.c_str(), "%f %f %f", &x, &y, &theta) != 3)
        {
            RCLCPP_WARN(this->get_logger(), "解析失败: '%s'", line.c_str());
            return;
        }

        // 构造 Odometry
        auto odom_msg = nav_msgs::msg::Odometry();          // 创建Odometry对象
        rclcpp::Time now = this->get_clock()->now();        // 创建时间对象用于消息时间戳或计算时间间隔
        odom_msg.header.stamp = now;                        // 构建时间辍
        odom_msg.header.frame_id = "odom";                  // 坐标系名称
        odom_msg.child_frame_id = "base_link";              // 子坐标系

        /* 机器人在父坐标系的位置 */
        odom_msg.pose.pose.position.x = x;                  // x坐标
        odom_msg.pose.pose.position.y = y;                  // y坐标
        odom_msg.pose.pose.position.z = 0.0;                // z坐标


        tf2::Quaternion q;                                  // 创建四元数对象
        q.setRPY(0.0, 0.0, theta);                          // 将欧拉角转换为四元数
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        // 差分计算速度
        double cur_time = now.seconds();
        double dt = cur_time - last_time_;
        if (dt > 1e-6)
        {
            odom_msg.twist.twist.linear.x = (x - last_x_) / dt;
            odom_msg.twist.twist.linear.y = (y - last_y_) / dt;
            odom_msg.twist.twist.angular.z = (theta - last_theta_) / dt;
        }
        last_time_ = cur_time;
        last_x_ = x;
        last_y_ = y;
        last_theta_ = theta;

        // 发布 Odometry
        odom_pub_->publish(odom_msg);

        // 广播 TF
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);

        RCLCPP_INFO(this->get_logger(), "Odometry发布: x=%.3f y=%.3f θ=%.3f", x, y, theta);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
