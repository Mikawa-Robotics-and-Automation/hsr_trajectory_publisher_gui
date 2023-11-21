#pragma once

#include <sensor_msgs/msg/joint_state.hpp>

namespace hsr_trajectory_publisher_gui
{

static constexpr int SLIDER_MAX = 1000;

static double get_joint_state(sensor_msgs::msg::JointState::SharedPtr msg, std::string key)
{
    auto it = std::find(msg->name.begin(), msg->name.end(), key);
    if (it == msg->name.end())
    {
        return 0.0;
    }
    return msg->position[std::distance(msg->name.begin(), it)];
}

static void set_slider_value(QSlider* slider, double value)
{
    slider->setValue(SLIDER_MAX * value);
}

static void set_joint_state_to_slider(QSlider* slider, sensor_msgs::msg::JointState::SharedPtr msg, std::string key)
{
    set_slider_value(slider, get_joint_state(msg, key));
}

static double get_slider_value(QSlider* slider)
{
    return slider->value() / static_cast<double>(SLIDER_MAX);
}

static void set_slider_limit(QSlider *slider, double min, double max)
{
    slider->setMinimum(min * SLIDER_MAX);
    slider->setMaximum(max * SLIDER_MAX);
}

}