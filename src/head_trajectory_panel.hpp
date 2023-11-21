#pragma once

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <QtWidgets>
#endif
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace hsr_trajectory_publisher_gui
{

class HeadTrajectoryPanel : public rviz_common::Panel
{
    Q_OBJECT
public:
    HeadTrajectoryPanel(QWidget *parent = nullptr);

    virtual void onInitialize();
    virtual void load(const rviz_common::Config &config);
    virtual void save(rviz_common::Config config) const;

public Q_SLOTS:
    void tick();
    void click_get_state();

protected:
    rclcpp::Node::SharedPtr nh_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscliber_;
    sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;

    QCheckBox* enable_check_;
    QPushButton* get_state_button_;
    QSlider* pan_slider_;
    QLabel* image_label_;
    QSlider* tilt_slider_;
};

}