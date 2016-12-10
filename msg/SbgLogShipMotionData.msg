# /*!
#  * Structure that stores data for the SBG_ECOM_LOG_SHIP_MOTION or SBG_ECOM_LOG_SHIP_MOTION_HP message. <br>
#  * The data are expressed in the standard NED Ekinox coordiante frame. 
#  * Surge is positive forward, sway is positive right and heave is positive down. <br>
#  * Note that status flag should be read before using the different parameters because it will provide validity information
#  * about all included outputs. Some frames may not provide the heave period or surge/sway axes for example
#  */
# typedef struct _SbgLogShipMotionData
# {
# uint32  timeStamp;                  /*!< Time in us since the sensor power up. */
# uint16  status;                     /*!< Ship Motion data status bitmask */
# float   mainHeavePeriod;            /*!< Main heave period in seconds. */
# float   shipMotion[3];              /*!< Surge, sway and heave in meters. */
# float   shipAccel[3];               /*!< Surge, sway and heave ship Acceleration in m.s^-2. */
# float   shipVel[3];                 /*!< Surge, sway and heave velocities */
# } SbgLogShipMotionData;

uint32      timeStamp
uint16      status
float32     mainHeavePeriod
float32[3]  shipMotion
float32[3]  shipAccel
float32[3]  shipVel
