#include "sbglogparser/boost/SBGLogtoROSPublisher.h"

//#include "SBGLogtoROSMsg.h"
//#include "sbglogparser2_visitor.h"

SBGLogtoROSPublisher::SBGLogtoROSPublisher(ros::NodeHandle _n,
                                           ros::NodeHandle _private_nh):
    n(_n),
    private_nh(_private_nh)
{
    pVisitor.reset(new visitor_sbglog_to_ros());
}
