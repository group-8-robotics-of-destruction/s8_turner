#include <ros/ros.h>

#include <s8_turner/turner_node.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <s8_msgs/Orientation.h>
#include <geometry_msgs/Twist.h>
#include <s8_common_node/Node.h>
#include <s8_turner/TurnAction.h>
#include <s8_motor_controller/StopAction.h>

#define PARAM_NAME_SPEED        "speed"
#define PARAM_DEFAULT_SPEED     1.5

using namespace s8;
using namespace s8::turner_node;

typedef motor_controller_node::RotationDirection RotationDirection;

class Turner : public Node {
private:
    ros::Subscriber imu_subscriber;
    ros::Publisher twist_publisher;
    actionlib::SimpleActionServer<s8_turner::TurnAction> turn_action;
    actionlib::SimpleActionClient<s8_motor_controller::StopAction> stop_action;

    int start_z;
    int latest_z;
    int desired_z;
    bool turning;
    double speed;
    RotationDirection direction;

public:
    Turner() : turning(false), stop_action(ACTION_STOP, true), turn_action(nh, ACTION_TURN, boost::bind(&Turner::action_execute_turn_callback, this, _1), false) {
        init_params();
        print_params();
        imu_subscriber = nh.subscribe<s8_msgs::Orientation>(TOPIC_ORIENTATION, 1, &Turner::imu_callback, this);
        twist_publisher = nh.advertise<geometry_msgs::Twist>(TOPIC_TWIST, 1);
        turn_action.start();

        ROS_INFO("Waiting for stop action server...");
        stop_action.waitForServer();
        ROS_INFO("Connected to stop action server!");
    }

private:
    void action_execute_turn_callback(const s8_turner::TurnGoalConstPtr & goal) {
        if(turning) {
            ROS_FATAL("Warning: Turning callback called when already turning");
        }

        turning = true;
        start_z = latest_z;
        desired_z = start_z + goal->degrees;
        int starting_z = start_z;

        if(goal->degrees > 0) {
            direction = RotationDirection::LEFT;
        } else {
            direction = RotationDirection::RIGHT;
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

        turning = false;
    }

    void update() {
        if(!turning) {
            return;
        }

        int diff = (start_z - latest_z);

        ROS_INFO("start_z: %d, latest_z: %d, diff: %d, desired_z: %d", start_z, latest_z, diff, desired_z);

        const int treshold_range = 5;

        int low_treshold = desired_z - treshold_range;
        int high_treshold = desired_z + treshold_range;

        if(latest_z >= low_treshold && latest_z <= high_treshold) {
            turning = false;
            ROS_INFO("Done turning.");
            stop();
            return;
        }
//        publish(direction*calculate_speed(std::abs(diff)));
	    publish(direction * speed);
    }

    void stop() {
        ROS_INFO("Stopping...");

        s8_motor_controller::StopGoal goal;
        goal.stop = true;
        stop_action.sendGoal(goal);

        bool finised_before_timeout = stop_action.waitForResult(ros::Duration(30.0));

        if(finised_before_timeout) {
            actionlib::SimpleClientGoalState state = stop_action.getState();
            ROS_INFO("Stop action finished. %s", state.toString().c_str());
        } else {
            ROS_WARN("Stop action timed out.");
        }
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
