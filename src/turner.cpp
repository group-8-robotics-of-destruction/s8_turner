#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <s8_msgs/Orientation.h>
#include <geometry_msgs/Twist.h>
#include <s8_common_node/Node.h>
#include <s8_turner/TurnAction.h>

#define NODE_NAME               "s8_turner_node"

#define TOPIC_ORIENTATION       "/s8/orientation"
#define TOPIC_TWIST             "/s8/twist"
#define ACTION_TURN             "/s8/turn"

#define PARAM_NAME_SPEED        "speed"
#define PARAM_DEFAULT_SPEED     1.5

class Turner : public s8::Node {
public:
    enum Direction {
        LEFT = 1,
        RIGHT = -1
    };

private:
    ros::Subscriber imu_subscriber;
    ros::Publisher twist_publisher;
    actionlib::SimpleActionServer<s8_turner::TurnAction> turn_action;

    int start_z;
    int latest_z;
    int desired_z;
    bool turning;
    double speed;
    Direction direction;

public:
    Turner() : turning(false), turn_action(nh, ACTION_TURN, boost::bind(&Turner::action_execute_turn_callback, this, _1), false) {
        init_params();
        print_params();
        imu_subscriber = nh.subscribe<s8_msgs::Orientation>(TOPIC_ORIENTATION, 0, &Turner::imu_callback, this);
        twist_publisher = nh.advertise<geometry_msgs::Twist>(TOPIC_TWIST, 0);
        turn_action.start();
    }

    void turn(Direction dir) {
        if(turning) {
            ROS_WARN("Already turning. Ignoring incoming turn command.");
            return;
        }

        ROS_INFO("Turning %s", dir == Direction::LEFT ? "left" : "right");

        direction = dir;

        turning = true;
    }

private:
    void action_execute_turn_callback(const s8_turner::TurnGoalConstPtr & goal) {
        turning = true;
        start_z = latest_z;
        desired_z = start_z + goal->degrees;
        int starting_z = start_z;

        if(goal->degrees > 0) {
            direction = Direction::LEFT;
        } else {
            direction = Direction::RIGHT;
        }

        const int timeout = 10; // 10 seconds.
        const int rate_hz = 10;

        ros::Rate rate(rate_hz);

        int ticks = 0;

        ROS_INFO("Turn action started. Current z: %d, desired turn: %d, desired z: %d", start_z, goal->degrees, desired_z);

        while(turning && ticks <= timeout * rate_hz) {
            rate.sleep();
            ticks++;
        }

        if(ticks >= timeout * rate_hz) {
            ROS_WARN("Unable to turn. Turn action failed. Desired z: %d, actual z: %d, error: %d", desired_z, latest_z, desired_z - latest_z);
            s8_turner::TurnResult turn_action_result;
            turn_action_result.degrees = latest_z - starting_z;
            turn_action.setAborted(turn_action_result);
        } else {
            s8_turner::TurnResult turn_action_result;
            turn_action_result.degrees = latest_z - starting_z;
            turn_action.setSucceeded(turn_action_result);
            ROS_INFO("Turn action succeeded. Desired z: %d, actual z: %d, Overshoot: %d", desired_z, latest_z, latest_z - desired_z);
        }
    }

    void update() {
        int diff = (start_z - latest_z);

        ROS_INFO("start_z: %d, latest_z: %d, diff: %d", start_z, latest_z, diff);

        const int treshold_range = 5;

        int low_treshold = desired_z - treshold_range;
        int high_treshold = desired_z + treshold_range;

        if(latest_z >= low_treshold && latest_z <= high_treshold) {
            turning = false;
            ROS_INFO("Done turning.");
            publish(0);
            return;
        }
        publish(calculate_speed(std::abs(diff)));
    }

    void imu_callback(const s8_msgs::Orientation::ConstPtr & orientation) {
        int z = orientation->absolute_z;

        if(!turning) {
            start_z = z;
        }

        latest_z = z;

        if(turning) {
            update();
        }
    }

    double calculate_speed(int abs_diff){
        // Test and eventually change 30 to an angular parameter and generalise formula
        if (abs_diff <= 30){
            return (0.5 + 0.017*abs_diff) * speed;
        }
        else if (abs_diff > 30){
            return (1.25 - 0.0083*abs_diff) * speed;
        }
    }

    void publish(double w) {
        ROS_INFO("w: %lf", w);
        geometry_msgs::Twist twist;
        twist.angular.z = w;
        twist_publisher.publish(twist);
    }

    void init_params() {
        add_param(PARAM_NAME_SPEED, speed, PARAM_DEFAULT_SPEED);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);
    Turner turner;
    ros::spin();
    return 0;
}
