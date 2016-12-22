#include "sbglogparser/std/sbglogparser.h"

using namespace sbglogparser_std;

void SBGLogParser::publish(SbgEComMsgId msg,
                             const SbgBinaryLogData *pLogData) {
    if (map_msg_to_publish.count(msg)) {
        map_msg_to_publish.at(msg)(pLogData);
    }
    else {
        throw SbgEComMsgIdException(msg);
    }
}

SbgErrorCode SBGLogParser::onLogReceived(SbgEComHandle *pHandle,
                                           SbgEComClass msgClass,
                                           SbgEComMsgId msg,
                                           const SbgBinaryLogData *pLogData) {
    publish(msg, pLogData);
    return SBG_NO_ERROR;
}
