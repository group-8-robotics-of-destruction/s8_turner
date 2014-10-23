#include <ros/ros.h>
#include <s8_msgs/Orientation.h>
#include <geometry_msgs/Twist.h>
#include <s8_common_node/Node.h>

#define NODE_NAME               "s8_turner_node"

#define TOPIC_ORIENTATION       "/s8/orientation"
#define TOPIC_TWIST             "/s8/twist"

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
    int start_z;
    int latest_z;
    bool turning;
    double speed;
    Direction direction;
    bool first;

public:
    Turner() : turning(false), first(true) {
        init_params();
        print_params();
        imu_subscriber = nh.subscribe<s8_msgs::Orientation>(TOPIC_ORIENTATION, 0, &Turner::imu_callback, this);
        twist_publisher = nh.advertise<geometry_msgs::Twist>(TOPIC_TWIST, 0);
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
    void update() {
        int diff = (start_z - latest_z);

        ROS_INFO("start_z: %d, latest_z: %d, diff: %d", start_z, latest_z, diff);

        if(std::abs(diff) >= 85 && std::abs(diff) <= 95) {
            turning = false;
            ROS_INFO("Done turning.");
            publish(0);
            return;
        }

        publish((int)direction * speed);
    }

    void imu_callback(const s8_msgs::Orientation::ConstPtr & orientation) {
        int z = orientation->z;

        if(!turning) {
            start_z = transform_rotation(z);
        }

        if(first) {
            turn(Turner::Direction::LEFT);
            first = false;
        }

        latest_z = transform_rotation(z);

        if(start_z > 250) {
            if(latest_z < 100) {
                latest_z += 360;
            }
        }

        if(start_z < 110) {
            if(latest_z > 250) {
                latest_z -= 360;
            }
        }

        if(turning) {
            update();
        }
    }

    int transform_rotation(int z) {
        if(z < 0) {
            return z + 360;
        }

        return z;
    }

    void publish(double w) {
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
