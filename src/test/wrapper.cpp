#include "wrapper.h"

void WrapperSBG2ROS::publish(SbgEComMsgId msg,
                             const SbgBinaryLogData *pLogData) {
    if (map_msg_to_publish.count(msg)) {
        map_msg_to_publish.at(msg)(pLogData);
    }
    else {
        throw SbgEComMsgIdException(msg);
    }
}

SbgErrorCode WrapperSBG2ROS::onLogReceived(SbgEComHandle *pHandle,
                                           SbgEComClass msgClass,
                                           SbgEComMsgId msg,
                                           const SbgBinaryLogData *pLogData) {
    publish(msg, pLogData);
    return SBG_NO_ERROR;
}
