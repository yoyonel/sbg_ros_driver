#ifndef ISBGLOGPARSER_H
#define ISBGLOGPARSER_H

#include "ros/ros.h"    // for ros::NodeHandle
#include <sbgEComLib.h> // for SbgEComHandle, SbgEComClass, SbgEComMsgId, SbgBinaryLogData

class ISBGLogParser {
public:
    ISBGLogParser(ros::NodeHandle _n=ros::NodeHandle(),
                  ros::NodeHandle _private_nh=ros::NodeHandle("~")):
        n(_n),
        private_nh(_private_nh)
    { }

    // -------------------------------------------------------------------------
    // VIRTUAL PURE
    // -------------------------------------------------------------------------
    virtual SbgErrorCode onLogReceived(SbgEComHandle *pHandle,
                                       SbgEComClass msgClass,
                                       SbgEComMsgId msg,
                                       const SbgBinaryLogData *pLogData) = 0;
    virtual void init() = 0;
    virtual void publish() = 0;
    // -------------------------------------------------------------------------

protected:
    ros::NodeHandle n;
    ros::NodeHandle private_nh;
};

typedef boost::shared_ptr< ISBGLogParser > ISBGLogParserPtr;
typedef boost::shared_ptr< ISBGLogParser const> ISBGLogParserConstPtr;

#endif // ISBGLOGPARSER_H
