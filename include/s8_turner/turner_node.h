#ifndef __TURNER_NODE_H
#define __TURNER_NODE_H

#include <string>

#include <s8_orientation/orientation_node.h>
#include <s8_motor_controller/motor_controller_node.h>

namespace s8 {
    namespace turner_node {
        const std::string NODE_NAME =               "s8_turner_node";

        const std::string TOPIC_ORIENTATION =       s8::orientation_node::TOPIC_ORIENTATION;
        const std::string TOPIC_TWIST =             s8::motor_controller_node::TOPIC_TWIST;
        const std::string ACTION_STOP =             s8::motor_controller_node::ACTION_STOP;
        const std::string ACTION_TURN =             "/s8/turn";

        enum Direction {
            LEFT = 1,
            RIGHT = -1
        };

        std::string to_string(Direction direction) {
            switch(direction) {
                case Direction::LEFT: return "LEFT";
                case Direction::RIGHT: return "RIGHT";
            }

            return "UNKNOWN";
        }
    }
}

#endif
