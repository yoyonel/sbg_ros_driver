#include <string>
//#include <boost/variant/apply_visitor.hpp>

static inline std::string
demangled_type_info_name(const std::type_info&ti)
{
    int status = 0;
    return abi::__cxa_demangle(ti.name(),0,0,&status);
}

template<typename T>
ros::Publisher SBGLogtoROSPublisher::add_pub()
{
    // $  rostopic list
    // /SbgLogEkfEuler
    // /SbgLogEkfNav
    // /SbgLogShipMotion
    // /rosout
    // /rosout_agg
    auto ros_pub = n.advertise<T>( build_rostopic_name<T>(), 10 );
    map_sbglog_pub[ typeid(T) ] = ros_pub;
    return ros_pub;
}

template<typename T>
std::string SBGLogtoROSPublisher::build_rostopic_name(
        const std::string& _prefix_pattern,
        const std::string& _suffix_pattern,
        const std::string& _suffix_for_rostopic_name) const
{
//    // url: http://stackoverflow.com/questions/1055452/c-get-name-of-type-in-template
//    char const * name = typeid( T ).name();
//    //
//    // url: http://www.boost.org/doc/libs/master/libs/core/doc/html/core/demangle.html
//    const std::string& demangled_name = boost::core::demangle( name );
    const std::string& demangled_name = demangled_type_info_name( typeid(T) );
    //
    // url: http://www.cplusplus.com/reference/string/string/find/
    const std::size_t& found_prefix = demangled_name.find(_prefix_pattern);
    const std::size_t& found_suffix = demangled_name.find(_suffix_pattern);
    assert(found_prefix!=std::string::npos);
    assert(found_suffix!=std::string::npos);
    //
    // url: http://www.cplusplus.com/reference/string/string/substr/
    return demangled_name.substr(found_prefix, found_suffix - found_prefix) +
            _suffix_for_rostopic_name;
}

//template<typename TMsg, typename TVisitor>
//void SBGLogtoROSPublisher::publish(TMsg& _msg, const TVisitor& _visitor)
//{
//    boost::apply_visitor(_visitor, _msg);
//}

//template<typename TMsg>
//void SBGLogtoROSPublisher::publish(TMsg& _msg)
//{
//    boost::apply_visitor(*pVisitor.get(), _msg);
//}
