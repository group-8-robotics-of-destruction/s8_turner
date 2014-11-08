#ifndef __TURNER_NODE_H
#define __TURNER_NODE_H

#include <s8_motor_controller/motor_controller_node.h>
#include <string>

namespace s8 {
    namespace turner_node {
        const std::string NODE_NAME =                   "s8_turner_node";

        const std::string TOPIC_ORIENTATION =           "/s8/orientation";
        const std::string ACTION_TURN =                 "/s8/turn";

        const std::string TOPIC_TWIST =                 s8::motor_controller_node::TOPIC_TWIST;
        const std::string ACTION_STOP =                 s8::motor_controller_node::ACTION_STOP;
    }
}

#endif