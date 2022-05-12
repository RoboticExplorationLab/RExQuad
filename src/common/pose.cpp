#include "pose.hpp"

#include "string.h"

namespace rexquad {

void PoseToBytes(char* buf, const PoseMsg& pose) {
    memcpy(buf+1, &pose, sizeof(PoseMsg));
    buf[0] = PoseMsg::MsgID();
}

bool PoseFromBytes(PoseMsg& pose, const char* buf) {
    bool is_msg_id_correct = buf[0] == PoseMsg::MsgID();
    if (is_msg_id_correct) {
        memcpy(&pose, buf+1, sizeof(PoseMsg));
    }
    return is_msg_id_correct;
}

}  // namespace rexquad