# //----------------------------------------------------------------------//
# //- Device info definitions                                              -//
# //----------------------------------------------------------------------//

# /* Misc */
# #define SBG_ECOM_INFO_PRODUCT_CODE_LENGTH         (32)

# //----------------------------------------------------------------------//
# //- Device Info structure                                            -//
# //----------------------------------------------------------------------//

# /*!
#  * Helper structure to retrieve device info.
#  */
# typedef struct _SbgEComDeviceInfo
# {
#   uint8   productCode[SBG_ECOM_INFO_PRODUCT_CODE_LENGTH]; /*!< Human readable Product Code. */
#   uint32  serialNumber;                                   /*!< Device serial number */
#   uint32  calibationRev;                                  /*!< Calibration data revision */
#   uint16  calibrationYear;                                /*!< Device Calibration Year */
#   uint8   calibrationMonth;                               /*!< Device Calibration Month */
#   uint8   calibrationDay;                                 /*!< Device Calibration Day */
#   uint32  hardwareRev;                                    /*!< Device hardware revision */
#   uint32  firmwareRev;                                    /*!< Firmware revision */
# } SbgEComDeviceInfo;

uint8[32]   productCode
uint32  serialNumber
uint32  calibationRev
uint16  calibrationYear
uint8   calibrationMonth
uint8   calibrationDay
uint32  hardwareRev
uint32  firmwareRev