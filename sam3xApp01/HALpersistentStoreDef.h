/*Sonde general includes after implementatio { section
 *See PersStorM for access
 */
#ifndef HALpersistentStoreDef_H
#define HALpersistentStoreDef_H


#define sizeOfSensorConfig() sizeof(SdTyp01_t)

//#define pEEPROM ((PeiEepromInternal_t *)0)
#define getAddrEeInt(param) \
    ((uint32_t) &((PeiEepromInternal_t *)0)->param)
#define getAddrEe_hdSensorMan() \
    ((uint16_t)&((PeiEepromInternal_t *)0)->hdSensorManConfig)
#define getAddrEe_hdSensorManConfig(param) \
    ((uint16_t)&((PeiEepromInternal_t *)0)->hdSensorManConfig.sensors[param])
#define getAddrEe_hdSensorManConfigSensorsPhyId(snsrIdx) \
    ((uint16_t)&((PeiEepromInternal_t *)0)->hdSensorManConfig.sensors[snsrIdx].sensorIdType)
#define getAddrEe_hdSensorManConfigCalbr(param,ver) \
    ((uint16_t)&((PeiEepromInternal_t *)0)->hdSensorManConfig.sensors[param].calbr##ver)
#define getAddrEe_lastKnownTimeStatus() \
    ((uint16_t)&((PeiEepromInternal_t *)0)->lastKnownTime.status)
#define getAddrEe_lastKnownTimeEntry(param) \
    ((uint16_t)&((PeiEepromInternal_t *)0)->lastKnownTime.entry[param])
#define getAddrEeFifo(param) \
    ((uint16_t)&((PeiSnsrRdgsFifo_t *)0x400)->rec[param])
#define getAddrEeFifoSize(param) \
    ((uint16_t)&((PeiSnsrRdgsFifo_t *)0x400)->rec[param].size)
#define getAddrEeFifoMng(param) \
    ((uint16_t)&((PeiSnsrRdgsFifo_t *)0x400)->rec[param].mng)

#endif //HALpersistentStoreDef_H
