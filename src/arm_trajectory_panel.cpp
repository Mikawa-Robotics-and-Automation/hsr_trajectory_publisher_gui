#include "arm_trajectory_panel.hpp"

#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QPainter>
#include <QMouseEvent>
#include <QSizePolicy>
#include "utils.hpp"

namespace hsr_trajectory_publisher_gui
{

ArmTrajectoryPanel::ArmTrajectoryPanel(QWidget *parent)
    : rviz_common::Panel(parent)
{
    QVBoxLayout* layout = new QVBoxLayout;

    // Creating new UI elements
    enable_check_ = new QCheckBox("Enable");
    get_state_button_ = new QPushButton("Get State", this);
    
    arm_lift_slider_ = new QSlider(Qt::Vertical, this);
    arm_flex_slider_ = new QSlider(Qt::Horizontal, this);
    arm_roll_slider_ = new QSlider(Qt::Horizontal, this);
    wrist_flex_slider_ = new QSlider(Qt::Horizontal, this);
    wrist_roll_slider_ = new QSlider(Qt::Horizontal, this);

    set_slider_limit(arm_lift_slider_, 0.0, 0.5);
    set_slider_value(arm_lift_slider_, 0.0);
    set_slider_limit(arm_flex_slider_, -1.57, 0);
    set_slider_value(arm_flex_slider_, 0.0);
    set_slider_limit(arm_roll_slider_, -1.57, 1.57);
    set_slider_value(arm_roll_slider_, 0.0);
    set_slider_limit(wrist_flex_slider_, -1.57, 1.57);
    set_slider_value(wrist_flex_slider_, 0.0);
    set_slider_limit(wrist_roll_slider_, -1.57, 1.57);
    set_slider_value(wrist_roll_slider_, 0.0);

    // tilt_slider_ = new QSlider(Qt::Vertical, this);
    // set_slider_limit(tilt_slider_, -0.76, 0.76);
    // set_slider_value(tilt_slider_, 0.0);

    connect(get_state_button_, SIGNAL(clicked()), this, SLOT(click_get_state()));

    // First row: 'Enable' checkbox and 'Get State' button
    QHBoxLayout* top_row_layout = new QHBoxLayout;
    top_row_layout->addWidget(enable_check_);
    top_row_layout->addWidget(get_state_button_);
    layout->addLayout(top_row_layout);

    // Second row: Horizontal slider
    // layout->addWidget(pan_slider_);

    image_label_ = new QLabel(this);
    QImage image;
    try {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("hsr_trajectory_publisher_gui");
        std::string pic_file = package_share_directory + "/resource/pic/arm.png";
        image.load(pic_file.c_str());
        image = image.scaled(400, 200, Qt::KeepAspectRatio);
        image_label_->setPixmap(QPixmap::fromImage(image));
    } catch (const std::runtime_error& e) {

    }

    QVBoxLayout* slider_layout = new QVBoxLayout;
    slider_layout->addWidget(wrist_roll_slider_);
    slider_layout->addWidget(wrist_flex_slider_);
    slider_layout->addWidget(arm_roll_slider_);
    slider_layout->addWidget(arm_flex_slider_);

    QHBoxLayout* third_row_layout = new QHBoxLayout;
    third_row_layout->addWidget(arm_lift_slider_);
    third_row_layout->addWidget(image_label_);
    third_row_layout->addLayout(slider_layout);
    layout->addLayout(third_row_layout);

    // Setting the main layout
    setLayout(layout);

    QTimer* output_timer = new QTimer(this);
    connect(output_timer, SIGNAL(timeout()), this, SLOT(tick()));
    output_timer->start(100);
}

void ArmTrajectoryPanel::onInitialize()
{
    nh_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    joint_traj_publisher_ = nh_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_trajectory_controller/joint_trajectory", rclcpp::QoS(3));
    joint_state_subscliber_ = nh_->create_subscription<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(3),
        [&](sensor_msgs::msg::JointState::SharedPtr msg)
        {
            latest_joint_state_ = msg;
        });
}

void ArmTrajectoryPanel::tick()
{
    if (enable_check_->isChecked())
    {
        auto msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();

        // msg->header.stamp = nh_->now();

        msg->joint_names = {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};

        // double pan  = get_slider_value(pan_slider_);
        double arm_lift = get_slider_value(arm_lift_slider_);
        double arm_flex = get_slider_value(arm_flex_slider_);
        double arm_roll = get_slider_value(arm_roll_slider_);
        double wrist_flex = get_slider_value(wrist_flex_slider_);
        double wrist_roll = get_slider_value(wrist_roll_slider_);

        msg->points.resize(1);
        msg->points[0].positions = {arm_lift, arm_flex, arm_roll, wrist_flex, wrist_roll};
        // msg->points[0].time_from_start.sec = msg->header.stamp.sec;
        // msg->points[0].time_from_start.nanosec = msg->header.stamp.nanosec;
        joint_traj_publisher_->publish(*msg);
    }
}

void ArmTrajectoryPanel::click_get_state()
{
    if(latest_joint_state_)
    {
        set_joint_state_to_slider(arm_lift_slider_, latest_joint_state_, "arm_lift_joint");
        set_joint_state_to_slider(arm_flex_slider_, latest_joint_state_, "arm_flex_joint");
        set_joint_state_to_slider(arm_roll_slider_, latest_joint_state_, "arm_roll_joint");
        set_joint_state_to_slider(wrist_flex_slider_, latest_joint_state_, "wrist_flex_joint");
        set_joint_state_to_slider(wrist_roll_slider_, latest_joint_state_, "wrist_roll_joint");
    }
}

void ArmTrajectoryPanel::save(rviz_common::Config config) const
{
    rviz_common::Panel::save(config);
}

void ArmTrajectoryPanel::load(const rviz_common::Config &config)
{
    rviz_common::Panel::load(config);
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hsr_trajectory_publisher_gui::ArmTrajectoryPanel, rviz_common::Panel)