# /*!
#  * Stores global status data.
#  */
# typedef struct _SbgLogStatusData
# {
#   uint32  timeStamp;                                              /*!< Time in us since the sensor power up. */
#   uint16  generalStatus;                                          /*!< General status bitmask and enums. */
#   uint16  reserved1;                                              /*!< Reserved status field for future use */
#   uint32  comStatus;                                              /*!< Communication status bitmask and enums. */
#   uint32  aidingStatus;                                           /*!< Aiding equipments status bitmask and enums. */
#   uint32  reserved2;                                              /*!< Reserved status field for future use */
#   uint16  reserved3;                                              /*!< Reserved status field for future use */
# } SbgLogStatusData;

uint32  timeStamp
uint16  generalStatus
uint16  reserved1
uint32  comStatus
uint32  aidingStatus
uint32  reserved2
uint16  reserved3
