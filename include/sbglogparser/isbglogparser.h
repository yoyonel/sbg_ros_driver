#ifndef ISBGLOGPARSER_H
#define ISBGLOGPARSER_H

#include "ros/ros.h"    // for ros::NodeHandle
#include <sbgEComLib.h> // for SbgEComHandle, SbgEComClass, SbgEComMsgId, SbgBinaryLogData

#include <cxxabi.h>    // for namespace abi::

// urls:
// - http://stackoverflow.com/questions/19005744/cfilt-does-not-demangle-typeid-name
// - https://gcc.gnu.org/onlinedocs/libstdc++/libstdc++-html-USERS-4.3/a01696.html 
static inline std::string
demangled_type_info_name(const std::type_info&ti)
{
    int status = 0;
    return abi::__cxa_demangle(ti.name(),0,0,&status);
}

template<typename TRosMsg>
std::string build_topic_name(
        const std::string& _prefix_pattern="SbgLog",
        const std::string& _suffix_pattern="Data",
        const std::string& _suffix_for_rostopic_name="")
{
    // // url: http://stackoverflow.com/questions/1055452/c-get-name-of-type-in-template
    // char const * name = typeid( TRosMsg ).name();
    // //
    // // url: http://www.boost.org/doc/libs/master/libs/core/doc/html/core/demangle.html
    // const std::string& demangled_name = boost::core::demangle( name );
    const std::string& demangled_name = demangled_type_info_name( typeid(TRosMsg) );
    //
    // url: http://www.cplusplus.com/reference/string/string/find/
    const std::size_t& found_prefix = demangled_name.find(_prefix_pattern);
    const std::size_t& found_suffix = demangled_name.find(_suffix_pattern);
    assert(found_prefix!=std::string::npos);
    assert(found_suffix!=std::string::npos);
    //
    // url: http://www.cplusplus.com/reference/string/string/substr/
    return demangled_name.substr(found_prefix, found_suffix - found_prefix) + _suffix_for_rostopic_name;
}

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
