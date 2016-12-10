# typedef struct _SbgLogEkfNavData
# {
#   uint32  timeStamp;              /*!< Time in us since the sensor power up. */
#   float   velocity[3];            /*!< North, East, Down velocity in m.s^-1. */
#   float   velocityStdDev[3];      /*!< North, East, Down velocity 1 sigma standard deviation in m.s^-1. */
#   double  position[3];            /*!< Latitude, Longitude in degrees positive North and East.
#                                        Altitude above Mean Sea Level in meters. */
#   float   undulation;             /*!< Altitude difference between the geoid and the Ellipsoid in meters (Height above Ellipsoid = altitude + undulation). */
#   float   positionStdDev[3];      /*!< Latitude, longitude and altitude 1 sigma standard deviation in meters. */
#   uint32  status;                 /*!< EKF solution status bitmask and enum. */
# } SbgLogEkfNavData;

uint32      timeStamp
float32[3]  velocity
float32[3]  velocityStdDev
float64[3]  position

float32     undulation
float32[3]  positionStdDev
uint32      status
