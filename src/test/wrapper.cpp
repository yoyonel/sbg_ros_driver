#include "sbgECom.h"
#include "sbgEComIds.h"

#include <map>
#include <typeindex>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <cstring>   // for std::memcpy

namespace sbg_driver {
typedef struct _SbgLogStatusData
{
    uint32	timeStamp;												/*!< Time in us since the sensor power up. */
    uint16	generalStatus;											/*!< General status bitmask and enums. */
    uint16	reserved1;												/*!< Reserved status field for future use */
    uint32	comStatus;												/*!< Communication status bitmask and enums. */
    uint32	aidingStatus;											/*!< Aiding equipments status bitmask and enums. */
    uint32	reserved2;												/*!< Reserved status field for future use */
    uint16	reserved3;												/*!< Reserved status field for future use */
} SbgLogStatusData;

typedef struct {
    uint32	timeStamp;				/*!< Time in us since the sensor power up. */
    float	euler[3];				/*!< Roll, Pitch and Yaw angles in rad. */
    float	eulerStdDev[3];			/*!< Roll, Pitch and Yaw angles 1 sigma standard deviation in rad. */
    uint32	status;					/*!< EKF solution status bitmask and enum. */
} SbgLogEkfEulerData;
}

namespace ros {
class Publisher {
public:
    Publisher() {}

    template<typename TRosMsg>
    void publish(const TRosMsg& _ros_msg)
    {}
};
typedef std::shared_ptr<Publisher> sharedptr_Publisher;
}

//----------------------------------------------------------------------------------
// Association des enums log de sbg aux ros msg type par spécialisation de templates
//----------------------------------------------------------------------------------

template <SbgEComLog enum_log>
struct SBGLogEnum_Types {
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_STATUS> {
    typedef SbgLogStatusData t_sbglog;
    typedef sbg_driver::SbgLogStatusData t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_EKF_EULER> {
    typedef SbgLogEkfEulerData t_sbglog;
    typedef sbg_driver::SbgLogEkfEulerData t_rosmsg;
};

// ...

std::map<SbgEComLog, ros::sharedptr_Publisher> map_enum_rospub;


//----------------------------------------
template<SbgEComLog enum_log,
         typename TSbgLog=typename SBGLogEnum_Types<enum_log>::t_sbglog,
         typename TRosMsg=typename SBGLogEnum_Types<enum_log>::t_rosmsg>
void publish(const SbgBinaryLogData *pLogData)
{
    typedef const TSbgLog* cstptr_TSbgLog;

    // const cast à l'ancienne assurée par l'API C SBG
    // => on récupère les données (pointeur mémoire vers les données)
    cstptr_TSbgLog pLogData_casted = (cstptr_TSbgLog)(pLogData);

    // on instancie un ros message associé au log
    TRosMsg ros_msg;

    // copie des données (à la mano, avec memcpy)
    std::memcpy(&ros_msg,
                &pLogData_casted,
                sizeof(TSbgLog));

    // récupération du ROS Publisher associé au log
    ros::Publisher ros_pub;
//    try{
    if(map_enum_rospub.count(enum_log)) {
        ros_pub = *map_enum_rospub.at(enum_log);
        std::cout << "Log: " << enum_log <<
                     " -> Recuperation d'un ROS publisher" << std::endl;
    }
//    catch (const std::out_of_range& oor) {
    else {
        // le ROS Publisher n'est pas initialisé/utilisé
        // création et initialisation du ROS Publisher
        // n.advertise<TRosMsg>( build_rostopic_name<TRosMsg>(), 10 );
        ros_pub = ros::Publisher();
        std::cout << "Log: " << enum_log <<
                     " -> Creation d'un ROS publisher" << std::endl;

        // sauvegarde dans la map de la correspondance: enum log -> ros publisher
        map_enum_rospub[enum_log] = std::make_shared<ros::Publisher>(ros_pub);
    }
    std::cout << "Log: " << enum_log <<
                 " -> Publication du ROS message" << std::endl;
    ros_pub.publish(ros_msg);
}
//----------------------------------------
//----------------------------------------------------------------------------------

int main()
{
    SbgLogStatusData status = {0, 1, 2, 3, 4, 5, 6};
    SbgLogEkfEulerData euler = {0, {1, 2, 3}, {4, 5, 6}, 7};

    // 1ere utilisation des logs/ros_messages/ros_publishers
    publish<SBG_ECOM_LOG_STATUS>((SbgBinaryLogData*)(&status));
    publish<SBG_ECOM_LOG_EKF_EULER>((SbgBinaryLogData*)(&euler));

    // re-utilisation
    publish<SBG_ECOM_LOG_STATUS>((SbgBinaryLogData*)(&status));
}
