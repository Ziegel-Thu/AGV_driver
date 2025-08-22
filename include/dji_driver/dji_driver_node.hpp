#ifndef DJI_DRIVER_NODE_HPP_
#define DJI_DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include "dji_driver/dji_protocol.hpp"

namespace dji_driver
{

/**
 * @brief DJI麦克纳姆轮底盘驱动节点类
 * 
 * 该节点提供以下功能：
 * 1. 订阅cmd_vel话题，将速度命令转换为底盘控制指令
 * 2. 通过两个独立串口与底盘通信：一个发送控制指令，一个接收编码器数据
 * 3. 根据编码器数据计算里程计信息并发布odom话题
 */
class DJIDriverNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * 
     * 初始化节点，包括参数加载、串口初始化、订阅发布设置和定时器创建
     */
    explicit DJIDriverNode();

private:
    /**
     * @brief 初始化所有节点参数
     * 
     * 声明并获取节点运行所需的各项参数，包括串口配置、速度参数等
     */
    void init_parameters();
    
    /**
     * @brief 初始化串口连接
     * 
     * 根据配置参数初始化控制串口和编码器串口
     * @return bool 初始化是否成功，有任一串口失败则返回false
     */
    bool init_serial();
    
    /**
     * @brief 初始化订阅发布器
     * 
     * 创建cmd_vel的订阅及odom的发布器
     */
    void init_subscriptions();
    
    /**
     * @brief 初始化定时器
     * 
     * 创建用于定期读取编码器和发送控制命令的定时器
     */
    void init_timers();

    /**
     * @brief cmd_vel话题回调函数
     * 
     * 处理接收到的速度控制命令，应用速度限制和偏移量
     * @param msg 接收到的Twist消息指针
     */
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    /**
     * @brief 控制指令发送定时回调
     * 
     * 定期向控制串口发送最新的控制命令
     */
    void control_write_callback();

    /**
     * @brief 向串口发送控制数据
     * 
     * 将控制命令结构体写入串口
     * @param cmd 要发送的控制命令
     * @return bool 发送是否成功
     */
    bool write_control_data(const ControlCommand& cmd);
    
    // 控制串口参数
    std::string control_port_;          ///< 控制串口设备名
    int control_baud_rate_;             ///< 控制串口波特率
    double control_timeout_;            ///< 控制串口读写超时时间(秒)
    
    // 速度参数
    double vx_offset_;                  ///< X方向速度偏移量
    double vy_offset_;                  ///< Y方向速度偏移量
    double wz_offset_;                  ///< Z轴角速度偏移量
    double vx_max_;                     ///< X方向最大速度限制
    double vy_max_;                     ///< Y方向最大速度限制
    double wz_max_;                     ///< Z轴最大角速度限制
    
    // DJI底盘参数
    int16_t chassis_speed_range_;       ///< DJI底盘速度范围，用户可配置
    
    bool debug_mode_;                   ///< 调试模式开关
    bool protocol_debug_;               ///< 协议调试模式开关，用于打印16进制协议数据
    double debug_print_interval_;       ///< 调试信息打印间隔(秒)
    rclcpp::Time last_tx_print_time_;   ///< 上次发送数据打印时间

    // ROS2相关
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;      ///< cmd_vel话题订阅器
    rclcpp::TimerBase::SharedPtr control_write_timer_;                            ///< 控制指令发送定时器

    // 串口相关
    serial::Serial control_serial_;     ///< 用于发送控制命令的串口对象
    bool control_connected_;            ///< 控制串口连接状态标志
    ControlCommand current_cmd_;        ///< 当前的控制命令
};

} // namespace dji_driver

#endif // DJI_DRIVER_NODE_HPP_ 