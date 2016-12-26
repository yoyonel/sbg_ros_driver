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
        // ros_pub = n.advertise<TRosMsg>("toto", 10);

        // sauvegarde dans la map de la correspondance: enum log -> ros publisher
        map_enum_rospub[enum_log] = std::make_shared<ros::Publisher>(ros_pub);
    }

    return ros_pub;
}