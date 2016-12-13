#include "sbgTypes.h"
#include "sbgEComBinaryLogDebug.h"

/*!
 * EKF computed orientation using euler angles.
 */
typedef struct _SbgLogEkfEulerData
{
	uint32	timeStamp;				/*!< Time in us since the sensor power up. */
	float	euler[3];				/*!< Roll, Pitch and Yaw angles in rad. */
	float	eulerStdDev[3];			/*!< Roll, Pitch and Yaw angles 1 sigma standard deviation in rad. */
	uint32	status;					/*!< EKF solution status bitmask and enum. */
} SbgLogEkfEulerData;
