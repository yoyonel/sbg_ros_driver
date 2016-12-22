template<SbgEComLog enum_log,
         typename TSbgLog,
         typename TRosMsg>
void SBGLogParser::publish(const SbgBinaryLogData *pLogData, ros::NodeHandle& n)
{
    TRosMsg ros_msg = build_rosmsg<TSbgLog, TRosMsg>(pLogData);

    // récupération du ROS Publisher associé au log
    ros::Publisher ros_pub = get_rospub<TRosMsg>(enum_log, n);

    // Publication du message ROS
    ros_pub.publish(ros_msg);
}

template<typename TSbgLog, typename TRosMsg>
TRosMsg SBGLogParser::build_rosmsg(const SbgBinaryLogData *pLogData)
{
    typedef const TSbgLog* TSbgLogPtr;
    typedef const TSbgLog& TSbgLogRef;

    // const cast à l'ancienne assurée par l'API C SBG
    // => on récupère les données (pointeur mémoire vers les données)
    TSbgLogRef pLogDataCasted = *(TSbgLogPtr)(pLogData);

    // on instancie un ros message associé au log
    TRosMsg ros_msg;

    // copie des données (à la mano, avec memcpy)
    std::memcpy(&ros_msg,
                &(pLogDataCasted),
                sizeof(TSbgLog));

    return ros_msg;
}

template<typename TRosMsg>
ros::Publisher SBGLogParser::get_rospub(const SbgEComLog& enum_log,
                                        ros::NodeHandle& n)
{
    // récupération du ROS Publisher associé au log
    ros::Publisher ros_pub;

    if(map_enum_rospub.count(enum_log)) {
        ros_pub = *map_enum_rospub.at(enum_log);
    }
    else {
        // le ROS Publisher n'est pas initialisé/utilisé
        // création et initialisation du ROS Publisher
        ros_pub = n.advertise<TRosMsg>(build_topic_name<TRosMsg>(), 10);

        // sauvegarde dans la map de la correspondance: enum log -> ros publisher
        map_enum_rospub[enum_log] = std::make_shared<ros::Publisher>(ros_pub);
    }

    return ros_pub;
}

template<typename TRosMsg>
std::string SBGLogParser::build_topic_name(
        const std::string& _prefix_pattern,
        const std::string& _suffix_pattern,
        const std::string& _suffix_for_rostopic_name)
{
    // url: http://stackoverflow.com/questions/1055452/c-get-name-of-type-in-template
    char const * name = typeid( TRosMsg ).name();
    //
    // url: http://www.boost.org/doc/libs/master/libs/core/doc/html/core/demangle.html
    const std::string& demangled_name = boost::core::demangle( name );
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
