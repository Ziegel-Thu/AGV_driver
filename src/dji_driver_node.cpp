#include "dji_driver/dji_driver_node.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sstream>
#include <iomanip>

namespace dji_driver
{

/**
 * @brief 构造函数，初始化节点
 * 
 * 完成节点的初始化，包括：
 * 1. 加载参数
 * 2. 初始化串口连接
 * 3. 设置订阅发布器
 * 4. 创建定时器
 */
DJIDriverNode::DJIDriverNode()
: Node("dji_driver_node"), control_connected_(false), // encoder_connected_(false),
  last_tx_print_time_(this->now()) //, last_rx_print_time_(this->now())
{
    // 初始化控制命令结构体
    current_cmd_.sof = 0xA5;
    current_cmd_.data_length = 6;
    current_cmd_.seq = 0;
    current_cmd_.cmd_id = ROS_VEL_CMD_ID;
    current_cmd_.linear_x = 1024;  // 停止值
    current_cmd_.linear_y = 1024;  // 停止值
    current_cmd_.angular_z = 1024; // 停止值
    
    // 初始化参数
    init_parameters();
    
    // 初始化串口
    if (!init_serial()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial ports");
        // 即使串口初始化失败，节点仍然继续运行，可能后续会自动重连
    }
    
    // 初始化订阅和定时器
    init_subscriptions();
    init_timers();
    
    RCLCPP_INFO(this->get_logger(), "DJIDriverNode has been initialized");
    RCLCPP_INFO(this->get_logger(), "注意: 当前模式为仅发送控制命令，编码器数据接收功能已禁用");
}

/**
 * @brief 初始化参数
 * 
 * 声明并获取节点运行所需的各项参数：
 * 1. 串口配置参数（端口、波特率、超时）
 * 2. 速度参数（偏移量、最大值）
 * 3. 机器人物理参数 
 * 4. 调试模式开关
 */
void DJIDriverNode::init_parameters()
{
    // 读取控制串口参数
    this->declare_parameter("control_serial.port", "/dev/ttyUSB0");
    this->declare_parameter("control_serial.baud_rate", 115200);
    this->declare_parameter("control_serial.timeout", 0.1);
    
    // 读取速度参数
    this->declare_parameter("velocity.linear.x.offset", 0.0);
    this->declare_parameter("velocity.linear.x.max", 1.0);
    this->declare_parameter("velocity.linear.y.offset", 0.0);
    this->declare_parameter("velocity.linear.y.max", 1.0);
    this->declare_parameter("velocity.angular.z.offset", 0.0);
    this->declare_parameter("velocity.angular.z.max", 1.0);
    
    // 读取DJI底盘参数
    this->declare_parameter("dji_chassis.speed_range", 20);
    
    // 读取其他参数
    this->declare_parameter("debug", false);
    this->declare_parameter("protocol_debug", false);
    this->declare_parameter("debug_print_interval", 2.0);
    
    // 获取控制串口参数值
    control_port_ = this->get_parameter("control_serial.port").as_string();
    control_baud_rate_ = this->get_parameter("control_serial.baud_rate").as_int();
    control_timeout_ = this->get_parameter("control_serial.timeout").as_double();
    
    // 获取速度参数值
    vx_offset_ = this->get_parameter("velocity.linear.x.offset").as_double();
    vx_max_ = this->get_parameter("velocity.linear.x.max").as_double();
    vy_offset_ = this->get_parameter("velocity.linear.y.offset").as_double();
    vy_max_ = this->get_parameter("velocity.linear.y.max").as_double();
    wz_offset_ = this->get_parameter("velocity.angular.z.offset").as_double();
    wz_max_ = this->get_parameter("velocity.angular.z.max").as_double();
    
    // 获取DJI底盘参数
    chassis_speed_range_ = this->get_parameter("dji_chassis.speed_range").as_int();
    
    debug_mode_ = this->get_parameter("debug").as_bool();
    protocol_debug_ = this->get_parameter("protocol_debug").as_bool();
    debug_print_interval_ = this->get_parameter("debug_print_interval").as_double();
}

/**
 * @brief 初始化串口连接
 * 
 * 根据配置参数初始化控制串口和编码器串口：
 * 1. 设置串口参数（端口名、波特率、超时）
 * 2. 尝试打开串口
 * 3. 记录连接状态
 * 
 * @return bool 返回初始化结果，如果任一串口初始化失败则返回false
 */
bool DJIDriverNode::init_serial()
{
    bool success = true;
    
    // 初始化控制串口
    try {
        control_serial_.setPort(control_port_);
        control_serial_.setBaudrate(control_baud_rate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(static_cast<uint32_t>(control_timeout_ * 1000));
        control_serial_.setTimeout(timeout);
        control_serial_.open();
        control_connected_ = true;
        RCLCPP_INFO(this->get_logger(), "Control serial port %s opened successfully", control_port_.c_str());
    } catch (const serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open control serial port: %s", e.what());
        success = false;
    }
    
    // 初始化编码器串口 - 已禁用
    // try {
    //     encoder_serial_.setPort(encoder_port_);
    //     encoder_serial_.setBaudrate(encoder_baud_rate_);
    //     serial::Timeout timeout = serial::Timeout::simpleTimeout(static_cast<uint32_t>(encoder_timeout_ * 1000));
    //     encoder_serial_.setTimeout(timeout);
    //     encoder_serial_.open();
    //     encoder_connected_ = true;
    //     RCLCPP_INFO(this->get_logger(), "Encoder serial port %s opened successfully", encoder_port_.c_str());
    // } catch (const serial::IOException& e) {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to open encoder serial port: %s", e.what());
    //     success = false;
    // }
    
    return success;
}

/**
 * @brief 初始化订阅和发布
 * 
 * 创建ROS2话题的订阅器和发布器：
 * 1. 订阅cmd_vel话题，接收速度控制命令
 * 2. 创建odom话题发布器，用于发布里程计信息
 * 3. 初始化TF广播器
 */
void DJIDriverNode::init_subscriptions()
{
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&DJIDriverNode::cmd_vel_callback, this, std::placeholders::_1));
    // odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    
    // 初始化TF广播器
    // tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

/**
 * @brief 初始化定时器
 * 
 * 创建定时器用于周期性执行任务：
 * 1. 编码器读取定时器：10ms周期，用于从串口读取编码器数据
 * 2. 控制命令发送定时器：50ms周期，用于向串口发送控制命令
 */
void DJIDriverNode::init_timers()
{
    // 创建定时器用于读取编码器数据 - 禁用编码器读取
    // encoder_read_timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(10),
    //     std::bind(&DJIDriverNode::encoder_read_callback, this));
    
    // 创建定时器用于发送控制命令
    control_write_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&DJIDriverNode::control_write_callback, this));
}

/**
 * @brief 处理cmd_vel话题的回调函数
 * 
 * 当接收到新的速度命令时：
 * 1. 应用偏移量和限制最大值
 * 2. 将速度映射到DJI底盘的有效范围
 * 3. 更新当前命令结构体
 * 4. 可选地输出调试信息
 * 
 * @param msg 接收到的Twist消息指针
 */
void DJIDriverNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // 应用偏移量和限制最大值
    double vx = std::clamp(msg->linear.x + vx_offset_, -vx_max_, vx_max_);
    double vy = std::clamp(msg->linear.y + vy_offset_, -vy_max_, vy_max_);
    double wz = std::clamp(msg->angular.z + wz_offset_, -wz_max_, wz_max_);
    
    // 将标准化速度[-1.0, 1.0]映射到DJI底盘的速度范围
    // 修改为使用用户配置的速度范围
    auto mapToDJIRange = [this](double value, double max_value) -> int16_t {
        // 将值归一化到[-1.0, 1.0]范围
        double normalized = value / max_value;
        normalized = std::clamp(normalized, -1.0, 1.0);
        
        // 映射到配置的DJI范围
        return static_cast<int16_t>(DJI_SPEED_MIDDLE + normalized * chassis_speed_range_);
    };
    
    // 应用映射
    current_cmd_.linear_x = mapToDJIRange(vx, vx_max_);
    current_cmd_.linear_y = mapToDJIRange(vy, vy_max_);  // 反转y方向
    current_cmd_.angular_z = mapToDJIRange(-wz, wz_max_);  // 反转wz方向
    
    if (debug_mode_) {
        RCLCPP_INFO(this->get_logger(), 
            "CMD_VEL: vx=%.2f, vy=%.2f, wz=%.2f -> DJI: %d, %d, %d",
            vx, vy, wz, current_cmd_.linear_x, current_cmd_.linear_y, current_cmd_.angular_z);
    }
}

/**
 * @brief 编码器读取定时器回调函数
 * 
 * 定期执行以从串口读取编码器数据：
 * 1. 检查编码器串口连接状态
 * 2. 尝试读取编码器数据
 * 3. 处理读取到的数据
 * 
 * 注意：当前已禁用此功能
 */
// void DJIDriverNode::encoder_read_callback()
// {
//     // 禁用编码器数据读取功能
//     RCLCPP_INFO_ONCE(this->get_logger(), "Encoder read function is disabled");
//     return;
    
//     // if (!encoder_connected_) return;
    
//     // if (read_encoder_data()) {
//     //     process_encoder_data(latest_encoder_data_);
//     // }
// }

/**
 * @brief 控制命令发送定时器回调函数
 * 
 * 定期执行以向串口发送控制命令：
 * 1. 检查控制串口连接状态
 * 2. 递增序列号
 * 3. 计算当前命令的校验和
 * 4. 发送控制命令
 */
void DJIDriverNode::control_write_callback()
{
    if (!control_connected_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "控制串口未连接，尝试重新连接...");
        try {
            control_serial_.open();
            control_connected_ = true;
            RCLCPP_INFO(this->get_logger(), "控制串口重新连接成功");
        } catch (const serial::IOException& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "重连失败: %s", e.what());
            return;
        }
    }
    
    // 递增序列号
    current_cmd_.seq = (current_cmd_.seq + 1) & 0xFF;
    
    // 计算并填充校验和
    fill_checksums(current_cmd_);
    
    // 打印协议数据（如果启用且满足时间间隔要求）
    if (protocol_debug_) {
        rclcpp::Time now = this->now();
        double elapsed = (now - last_tx_print_time_).seconds();
        
        if (elapsed >= debug_print_interval_) {
            const uint8_t* data = reinterpret_cast<const uint8_t*>(&current_cmd_);
            std::stringstream ss;
            ss << "TX Control: ";
            for (size_t i = 0; i < sizeof(ControlCommand); ++i) {
                ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') 
                   << static_cast<int>(data[i]) << " ";
            }
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
            last_tx_print_time_ = now;
        }
    }
    
    try {
        if (!write_control_data(current_cmd_)) {
            RCLCPP_WARN(this->get_logger(), "发送控制命令失败，发送字节数不匹配");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "发送控制命令时发生异常: %s", e.what());
        control_connected_ = false;
    }
}

/**
 * @brief 向串口发送控制数据
 * 
 * 将控制命令结构体写入串口：
 * 1. 尝试写入数据
 * 2. 验证是否写入成功
 * 3. 处理可能的异常
 * 
 * @param cmd 要发送的控制命令结构体
 * @return bool 是否发送成功
 */
bool DJIDriverNode::write_control_data(const ControlCommand& cmd)
{
    try {
        // ===== [协议相关] 发送控制命令 =====
        size_t bytes_written = control_serial_.write(
            reinterpret_cast<const uint8_t*>(&cmd),
            sizeof(ControlCommand));
        return bytes_written == sizeof(ControlCommand);
        // ===================================
    } catch (const serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "Control serial write error: %s", e.what());
        control_connected_ = false;
        return false;
    }
}

} // namespace dji_driver

/**
 * @brief 主函数
 * 
 * ROS2节点的入口点：
 * 1. 初始化ROS2
 * 2. 创建节点实例
 * 3. 进入事件循环
 * 4. 退出前清理资源
 * 
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 程序退出码
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dji_driver::DJIDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
