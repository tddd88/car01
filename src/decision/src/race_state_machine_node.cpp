#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

enum class State {
    NAVIGATING,
    VISUAL_SERVO,
    SEARCHING_BLIND,
    FINISHED
};

class RaceStateMachine {
public:
    RaceStateMachine() : ac_("move_base", true) {
        target_sub_ = nh_.subscribe("/blue_target", 1, &RaceStateMachine::targetCb, this);
        vision_vel_sub_ = nh_.subscribe("/cmd_vel_vision", 1, &RaceStateMachine::visionVelCb, this);
        nav_vel_sub_ = nh_.subscribe("/cmd_vel_nav", 1, &RaceStateMachine::navVelCb, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        
        current_state_ = State::NAVIGATING;
        has_target_ = false;
        
        while(!ac_.waitForServer(ros::Duration(1.0)) && ros::ok()){
            ROS_INFO_THROTTLE(5, "Waiting for the move_base action server to come up");
        }
    }

    void targetCb(const geometry_msgs::Point::ConstPtr& msg) {
        has_target_ = true;
        last_target_time_ = ros::Time::now();
    }

    void visionVelCb(const geometry_msgs::Twist::ConstPtr& msg) {
        vision_vel_ = *msg;
    }

    void navVelCb(const geometry_msgs::Twist::ConstPtr& msg) {
        if (current_state_ == State::NAVIGATING) {
            cmd_pub_.publish(*msg);
        }
    }

    void run() {
        ros::Rate r(10);
        while(ros::ok()) {
            if (has_target_ && (ros::Time::now() - last_target_time_).toSec() > 1.0) {
                has_target_ = false;
            }

            switch(current_state_) {
                case State::NAVIGATING:
                    if (has_target_) {
                        ac_.cancelAllGoals();
                        current_state_ = State::VISUAL_SERVO;
                        ROS_INFO("Target Detected! Switching to VISUAL_SERVO");
                    }
                    break;
                
                case State::VISUAL_SERVO:
                    cmd_pub_.publish(vision_vel_);
                    if (!has_target_) {
                        current_state_ = State::SEARCHING_BLIND; 
                        ROS_WARN("Target Lost! Blind searching...");
                    }
                    if (std::abs(vision_vel_.linear.x) < 0.01 && std::abs(vision_vel_.linear.y) < 0.01 && has_target_) {
                         current_state_ = State::FINISHED;
                         ROS_INFO("Task Finished!");
                    }
                    break;

                case State::SEARCHING_BLIND: {
                    geometry_msgs::Twist search_vel;
                    search_vel.linear.y = 0.2; // 螃蟹步寻找
                    cmd_pub_.publish(search_vel);
                    if (has_target_) {
                        current_state_ = State::VISUAL_SERVO;
                        ROS_INFO("Target Re-captured!");
                    }
                    break;
                }
                    
                case State::FINISHED: {
                    geometry_msgs::Twist stop_vel;
                    cmd_pub_.publish(stop_vel);
                    break;
                }
            }
            ros::spinOnce();
            r.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber target_sub_, vision_vel_sub_, nav_vel_sub_;
    ros::Publisher cmd_pub_;
    MoveBaseClient ac_;
    State current_state_;
    bool has_target_;
    ros::Time last_target_time_;
    geometry_msgs::Twist vision_vel_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "race_state_machine_node");
    RaceStateMachine rsm;
    rsm.run();
    return 0;
}
