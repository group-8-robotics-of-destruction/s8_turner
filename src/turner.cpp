#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <s8_msgs/Orientation.h>
#include <geometry_msgs/Twist.h>
#include <s8_common_node/Node.h>
#include <s8_turner/TurnAction.h>
#include <s8_motor_controller/StopAction.h>

#define NODE_NAME               "s8_turner_node"

#define TOPIC_ORIENTATION       "/s8/orientation"
#define TOPIC_TWIST             "/s8/twist"
#define ACTION_TURN             "/s8/turn"
#define ACTION_STOP             "/s8_motor_controller/stop"

#define PARAM_NAME_KP_TURN      "kp_turn"
#define PARAM_DEFAULT_KP_TURN   1.5
#define PARAM_NAME_DRIFT_TIME   "drift_time"
#define PARAM_DEFAULT_DRIFT_TIME 0.5

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
    actionlib::SimpleActionClient<s8_motor_controller::StopAction> stop_action;

    int start_z;
    int latest_z;
    int desired_z;
    int drift_per_sample;
    int samples_from_start;
    bool turning;
    double kp_turn;
    double drift_time;
    Direction direction;

public:
    Turner() : drift_per_sample(0), samples_from_start(0), turning(false), stop_action(ACTION_STOP, true), turn_action(nh, ACTION_TURN, boost::bind(&Turner::action_execute_turn_callback, this, _1), false) {
        init_params();
        print_params();
        imu_subscriber = nh.subscribe<s8_msgs::Orientation>(TOPIC_ORIENTATION, 0, &Turner::imu_callback, this);
        twist_publisher = nh.advertise<geometry_msgs::Twist>(TOPIC_TWIST, 0);
        turn_action.start();

        ROS_INFO("Waiting for stop action server...");
        stop_action.waitForServer();
        ROS_INFO("Connected to stop action server!");
    }

private:
    void action_execute_turn_callback(const s8_turner::TurnGoalConstPtr & goal) {
        start_z = latest_z;

        if(goal->degrees > 0) {
            direction = Direction::LEFT;
        } else {
            direction = Direction::RIGHT;
        }

        const int timeout = 10; // 10 seconds.
        const int rate_hz = 10;

        ros::Rate rate(rate_hz);

        // Drift estimate before starting the actual turning action

        drift_per_sample = drift_estimate(drift_time, rate_hz, start_z, rate);

        turning = true;
        desired_z = start_z + goal->degrees;
        start_z = latest_z;
        int starting_z = start_z;

        ROS_INFO("Turn action started. Current z: %d, desired turn: %d, desired z: %d", start_z, goal->degrees, desired_z);

        while(turning && samples_from_start <= timeout * rate_hz) {
            ROS_INFO("Turn action started. Current z: %d, desired turn: %d, desired z: %d", start_z, goal->degrees, desired_z);
            rate.sleep();
            samples_from_start++;
        }

        if(samples_from_start >= timeout * rate_hz) {
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
        publish(calculate_speed(std::abs(diff)));
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
        return (kp_turn * abs_diff) - drift_per_sample * samples_from_start;
    }

    // Check type, is int ok or should it be double?
    int drift_estimate(double drift_time, int rate_hz, int starting_z, ros::Rate rate) {
        int samples = 0;
        while (turning && samples < drift_time * rate_hz) {
            ROS_INFO("Estimating drift: N %d start_z: %d, total diff: %d", samples, starting_z, latest_z - starting_z);
            rate.sleep();
            samples++;
        }
        return (latest_z - starting_z)/samples;
    }

    void publish(double w) {
        ROS_INFO("w: %lf", w);
        geometry_msgs::Twist twist;
        twist.angular.z = w;
        twist_publisher.publish(twist);
    }

    void init_params() {
        add_param(PARAM_NAME_KP_TURN, kp_turn, PARAM_DEFAULT_KP_TURN);
        add_param(PARAM_NAME_DRIFT_TIME, drift_time, PARAM_DEFAULT_DRIFT_TIME);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);
    Turner turner;
    ros::spin();
    return 0;
}
