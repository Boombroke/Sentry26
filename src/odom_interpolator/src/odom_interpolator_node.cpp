#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <deque>
#include <memory>
#include <cmath>
#include <string>
#include <sstream>

class OdomInterpolator:public rclcpp::Node
{
public:
    OdomInterpolator():Node("odom_interpolator")
    {
        this->declare_parameter<std::string>("odom_topic","/odometry");
        this->declare_parameter<std::string>("interpolated_odom_topic","/odom_interpolated");
        this->declare_parameter<std::string>("diff_vel_topic","/cmd_vel_diff");
        this->declare_parameter<std::string>("cmd_vel_topic","/cmd_vel");
        this->declare_parameter<std::string>("velocity_marker_topic","/velocity_markers");
        this->declare_parameter<double>("interpolation_rate",100.0);
        this->declare_parameter<double>("arrow_scale",1.0);

        this->get_parameter("odom_topic",odom_topic_);
        this->get_parameter("interpolated_odom_topic",interpolated_odom_topic_);
        this->get_parameter("diff_vel_topic",diff_vel_topic_);
        this->get_parameter("cmd_vel_topic",cmd_vel_topic_);
        this->get_parameter("velocity_marker_topic",velocity_marker_topic_);
        this->get_parameter("interpolation_rate",interpolation_rate_);
        this->get_parameter("arrow_scale",arrow_scale_);

        //接收odom
        odom_sub_=this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_,10,
            std::bind(&OdomInterpolator::odomCallback,this,std::placeholders::_1));

        cmd_vel_sub_=this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_,10,
            std::bind(&OdomInterpolator::cmdVelCallback,this,std::placeholders::_1));

        //发布插值的odom
        interpolated_odom_pub_=this->create_publisher<nav_msgs::msg::Odometry>(
            interpolated_odom_topic_,10);
            
        //发布差速后的速度
        diff_vel_pub_=this->create_publisher<geometry_msgs::msg::Twist>(
            diff_vel_topic_, 10);

        //发布速度标记（使用MarkerArray一次性发布多个箭头）
        velocity_marker_pub_=this->create_publisher<visualization_msgs::msg::MarkerArray>(
            velocity_marker_topic_, 10);

        last_odom_time_=this->now();
        last_odom_stamp_=this->now();
        
        // 初始化速度
        actual_velocity_.linear.x = 0.0;
        actual_velocity_.linear.y = 0.0;
        actual_velocity_.linear.z = 0.0;
        actual_velocity_.angular.z = 0.0;
        
        cmd_velocity_.linear.x = 0.0;
        cmd_velocity_.linear.y = 0.0;
        cmd_velocity_.linear.z = 0.0;
        // 线性插值姿态(四元数球面线性插值)
        cmd_velocity_.angular.z = 0.0;
        
        // 初始化位置和姿态
        current_position_.x = 0.0;
        current_position_.y = 0.0;
        current_position_.z = 0.0;
        current_orientation_.x = 0.0;
        current_orientation_.y = 0.0;
        current_orientation_.z = 0.0;
        current_orientation_.w = 1.0;
    }

private:

    // odom回调
    void  odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 更新当前机器人位置和姿态（用于绘制箭头）
        current_position_ = msg->pose.pose.position;
        current_orientation_ = msg->pose.pose.orientation;
        
        // 计算实际速度（从里程计位置差除以dt）
        rclcpp::Time current_stamp = rclcpp::Time(msg->header.stamp);
        double dt = (current_stamp - last_odom_stamp_).seconds();
        
        if (dt > 0.001 && odom_buffer_.size() > 0) // 确保有有效的时间差和之前的数据
        {
            // 获取上一个里程计数据
            nav_msgs::msg::Odometry last_odom = odom_buffer_.back();
            
            // 计算位置差
            double dx = msg->pose.pose.position.x - last_odom.pose.pose.position.x;
            double dy = msg->pose.pose.position.y - last_odom.pose.pose.position.y;
            double dz = msg->pose.pose.position.z - last_odom.pose.pose.position.z;
            
            // 计算实际速度（路程除以dt）
            actual_velocity_.linear.x = dx / dt;
            actual_velocity_.linear.y = dy / dt;
            actual_velocity_.linear.z = dz / dt;
            
            // 计算角速度
            tf2::Quaternion q_prev, q_curr;
            tf2::fromMsg(last_odom.pose.pose.orientation, q_prev);
            tf2::fromMsg(msg->pose.pose.orientation, q_curr);
            
            // 获取yaw角度
            tf2::Matrix3x3 m_prev(q_prev);
            tf2::Matrix3x3 m_curr(q_curr);
            double roll_prev, pitch_prev, yaw_prev;
            double roll_curr, pitch_curr, yaw_curr;
            m_prev.getRPY(roll_prev, pitch_prev, yaw_prev);
            m_curr.getRPY(roll_curr, pitch_curr, yaw_curr);
            
            // 处理角度跳变
            double yaw_diff = yaw_curr - yaw_prev;
            if (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
            if (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
            
            actual_velocity_.angular.z = yaw_diff / dt;
        }
        
        // 缓存里程计数据
        odom_buffer_.push_back(*msg);

        // 保持缓冲区大小
        if (odom_buffer_.size() > 10)
        {
            odom_buffer_.pop_front();
        }

        rclcpp::Time current_time = this->now();
        rclcpp::Duration dt_pub = current_time - last_odom_time_;

        if (dt_pub.seconds() >= 1.0 / interpolation_rate_)
        {
            interpolateAndPublish();
            publishVelocityMarkers();
            last_odom_time_ = current_time;
        }
        
        last_odom_stamp_ = current_stamp;
    }
    
    // cmd_vel回调
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 接收cmd_vel（发布给下位的速度命令）
        cmd_velocity_ = *msg;
    }


    // 插值并发布
    void interpolateAndPublish()
    {
        // 确保缓冲区有足够数据
        if (odom_buffer_.size() < 2)
        {
            return;
        }

        // 获取最近的两个里程计数据
        nav_msgs::msg::Odometry odom_prev = odom_buffer_[odom_buffer_.size() - 2];
        nav_msgs::msg::Odometry odom_curr = odom_buffer_[odom_buffer_.size() - 1];

        // 计算时间差
        double dt = (rclcpp::Time(odom_curr.header.stamp) - rclcpp::Time(odom_prev.header.stamp)).seconds();

        // 计算速度（位置差分）
        double vx = (odom_curr.pose.pose.position.x - odom_prev.pose.pose.position.x) / dt;
        double vy = (odom_curr.pose.pose.position.y - odom_prev.pose.pose.position.y) / dt;
        double vz = (odom_curr.pose.pose.position.z - odom_prev.pose.pose.position.z) / dt;

        // 计算角速度（姿态差分）
        // tf2::Quaternion q_prev, q_curr;
        // tf2::fromMsg(odom_prev.pose.pose.orientation, q_prev);
        // tf2::fromMsg(odom_curr.pose.pose.orientation, q_curr);
        // tf2::Quaternion q_diff = q_curr * q_prev.inverse();
        // tf2::Matrix3x3 m_diff(q_diff);
        // double roll_diff, pitch_diff, yaw_diff;
        // m_diff.getRPY(roll_diff, pitch_diff, yaw_diff);
        // double v_roll = roll_diff / dt;
        // double v_pitch = pitch_diff / dt;
        // double v_yaw = yaw_diff / dt;

        // 插值位姿
        // 线性插值位置
        rclcpp::Time current_time=this->now();
        double interpolation_factor = (current_time - rclcpp::Time(odom_prev.header.stamp)).seconds() / dt;
        nav_msgs::msg::Odometry interpolated_odom = odom_prev;
        interpolated_odom.header.stamp = current_time;
        interpolated_odom.pose.pose.position.x = odom_prev.pose.pose.position.x + vx * (current_time - rclcpp::Time(odom_prev.header.stamp)).seconds();
        interpolated_odom.pose.pose.position.y = odom_prev.pose.pose.position.y + vy * (current_time - rclcpp::Time(odom_prev.header.stamp)).seconds();
        interpolated_odom.pose.pose.position.z = odom_prev.pose.pose.position.z + vz * (current_time - rclcpp::Time(odom_prev.header.stamp)).seconds();

        // 线性插值姿态(四元数球面线性插值)
        // tf2::Quaternion q_interpolated = q_prev.slerp(q_curr, interpolation_factor);
        // tf2::toTFToMsg(q_interpolated, interpolated_odom.pose.pose.orientation);

        // 设置速度
        interpolated_odom.twist.twist.linear.x = vx;
        interpolated_odom.twist.twist.linear.y = vy;
        interpolated_odom.twist.twist.linear.z = vz;
        // interpolated_odom.twist.twist.angular.x = v_roll;
        // interpolated_odom.twist.twist.angular.y = v_pitch;
        // interpolated_odom.twist.twist.angular.z = v_yaw;

        geometry_msgs::msg::Twist cmd_vel_world;
        cmd_vel_world.linear.x = vx;
        cmd_vel_world.linear.y = vy;

        // 发布插值后的里程计数据
        interpolated_odom_pub_->publish(interpolated_odom);
        diff_vel_pub_->publish(cmd_vel_world);
    }
    
    // 发布速度标记
    void publishVelocityMarkers()
    {
        // 如果没有里程计数据，不发布标记
        if (odom_buffer_.empty())
        {
            return;
        }
        // 线性插值姿态(四元数球面线性插值)
        
        // 获取最新的里程计数据以获取frame_id
        nav_msgs::msg::Odometry latest_odom = odom_buffer_.back();
        std::string frame_id = latest_odom.header.frame_id;
        if (frame_id.empty())
        {
            frame_id = "odom"; // 默认frame_id
        }
        
        // 计算速度模长
        double actual_speed = std::sqrt(
            actual_velocity_.linear.x * actual_velocity_.linear.x +
            actual_velocity_.linear.y * actual_velocity_.linear.y +
            actual_velocity_.linear.z * actual_velocity_.linear.z);

        tf2::Quaternion q;
        tf2::fromMsg(current_orientation_, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double cmd_vx_world = cmd_velocity_.linear.x * std::cos(yaw) - cmd_velocity_.linear.y * std::sin(yaw);
        double cmd_vy_world = cmd_velocity_.linear.x * std::sin(yaw) + cmd_velocity_.linear.y * std::cos(yaw);
        double cmd_speed = std::sqrt(
            cmd_vx_world * cmd_vx_world +
            cmd_vy_world * cmd_vy_world +
            cmd_velocity_.linear.z * cmd_velocity_.linear.z);

        // 发布实际速度箭头（绿色）
        visualization_msgs::msg::Marker actual_vel_marker;
        actual_vel_marker.header.frame_id = frame_id;
        actual_vel_marker.header.stamp = this->now();
        actual_vel_marker.ns = "velocity_arrow";
        actual_vel_marker.id = 0;
        actual_vel_marker.type = visualization_msgs::msg::Marker::ARROW;
        actual_vel_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // 箭头起点（当前位置）
        geometry_msgs::msg::Point start;
        start.x = current_position_.x;
        start.y = current_position_.y;
        start.z = current_position_.z;
        
        // 箭头终点（当前位置 + 速度向量）
        geometry_msgs::msg::Point end;
        end.x = current_position_.x + actual_velocity_.linear.x * arrow_scale_;
        end.y = current_position_.y + actual_velocity_.linear.y * arrow_scale_;
        end.z = current_position_.z + actual_velocity_.linear.z * arrow_scale_;
        
        actual_vel_marker.points.push_back(start);
        actual_vel_marker.points.push_back(end);
        
        // 设置箭头属性
        actual_vel_marker.scale.x = 0.07; // 箭头 shaft diameter
        actual_vel_marker.scale.y = 0.12; // 箭头 head diameter
        actual_vel_marker.scale.z = 0.2; // 箭头 head length
        actual_vel_marker.color.a = 1.0;
        actual_vel_marker.color.r = 0.0;
        actual_vel_marker.color.g = 1.0; // 绿色表示实际速度
        actual_vel_marker.color.b = 0.0;
        
        // 速度文本（实际速度）
        visualization_msgs::msg::Marker actual_text_marker;
        actual_text_marker.header.frame_id = frame_id;
        actual_text_marker.header.stamp = actual_vel_marker.header.stamp;
        actual_text_marker.ns = "velocity_text";
        actual_text_marker.id = 0;
        actual_text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        actual_text_marker.action = visualization_msgs::msg::Marker::ADD;
        actual_text_marker.pose.position = end;
        actual_text_marker.pose.position.z += 0.15;
        actual_text_marker.pose.orientation.w = 1.0;
        actual_text_marker.scale.z = 0.2;
        actual_text_marker.color.a = 1.0;
        actual_text_marker.color.r = 0.0;
        actual_text_marker.color.g = 1.0;
        actual_text_marker.color.b = 0.0;
        actual_text_marker.text = "odom: " + formatVelocityString(actual_speed);

        // 发布cmd_vel速度箭头（红色）
        visualization_msgs::msg::Marker cmd_vel_marker;
        cmd_vel_marker.header.frame_id = frame_id;
        cmd_vel_marker.header.stamp = this->now();
        cmd_vel_marker.ns = "velocity_arrow";
        cmd_vel_marker.id = 1;
        cmd_vel_marker.type = visualization_msgs::msg::Marker::ARROW;
        cmd_vel_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // 箭头起点（当前位置，稍微偏移以避免重叠）
        geometry_msgs::msg::Point cmd_start;
        cmd_start.x = current_position_.x;
        cmd_start.y = current_position_.y;
        cmd_start.z = current_position_.z + 0.2; // 稍微抬高
        
        // 箭头终点
        geometry_msgs::msg::Point cmd_end;
        cmd_end.x = current_position_.x + cmd_vx_world * arrow_scale_;
        cmd_end.y = current_position_.y + cmd_vy_world * arrow_scale_;
        cmd_end.z = current_position_.z + 0.2;
        
        cmd_vel_marker.points.push_back(cmd_start);
        cmd_vel_marker.points.push_back(cmd_end);
        
        // 设置箭头属性
        cmd_vel_marker.scale.x = 0.07;
        cmd_vel_marker.scale.y = 0.12;
        cmd_vel_marker.scale.z = 0.2;
        cmd_vel_marker.color.a = 1.0;
        cmd_vel_marker.color.r = 1.0; // 红色表示cmd_vel速度
        cmd_vel_marker.color.g = 0.0;
        cmd_vel_marker.color.b = 0.0;
        
        // 速度文本（命令速度）
        visualization_msgs::msg::Marker cmd_text_marker;
        cmd_text_marker.header.frame_id = frame_id;
        cmd_text_marker.header.stamp = cmd_vel_marker.header.stamp;
        cmd_text_marker.ns = "velocity_text";
        cmd_text_marker.id = 1;
        cmd_text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        cmd_text_marker.action = visualization_msgs::msg::Marker::ADD;
        cmd_text_marker.pose.position = cmd_end;
        cmd_text_marker.pose.position.z += 0.15;
        cmd_text_marker.pose.orientation.w = 1.0;
        cmd_text_marker.scale.z = 0.2;
        cmd_text_marker.color.a = 1.0;
        cmd_text_marker.color.r = 1.0;
        cmd_text_marker.color.g = 0.0;
        cmd_text_marker.color.b = 0.0;
        cmd_text_marker.text = "cmd: " + formatVelocityString(cmd_speed);

        // 创建MarkerArray并添加所有箭头
        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(actual_vel_marker);
        marker_array.markers.push_back(cmd_vel_marker);
        marker_array.markers.push_back(actual_text_marker);
        marker_array.markers.push_back(cmd_text_marker);
        
        // 一次性发布所有标记
        velocity_marker_pub_->publish(marker_array);
    }

    std::string formatVelocityString(double speed) const
    {
        std::ostringstream stream;
        stream.setf(std::ios::fixed, std::ios::floatfield);
        stream.precision(2);
        stream << speed << " m/s";
        return stream.str();
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr interpolated_odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr diff_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr velocity_marker_pub_;
    std::string odom_topic_;
    std::string interpolated_odom_topic_, diff_vel_topic_;
    std::string cmd_vel_topic_;
    std::string velocity_marker_topic_;
    double interpolation_rate_;
    double arrow_scale_;
    rclcpp::Time last_odom_time_;
    rclcpp::Time last_odom_stamp_;
    std::deque<nav_msgs::msg::Odometry> odom_buffer_; // 里程计数据缓冲区
    geometry_msgs::msg::Twist actual_velocity_; // 实际速度
    geometry_msgs::msg::Twist cmd_velocity_; // cmd_vel速度命令
    geometry_msgs::msg::Point current_position_; // 当前位置
    geometry_msgs::msg::Quaternion current_orientation_; // 当前姿态
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node= std::make_shared<OdomInterpolator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}