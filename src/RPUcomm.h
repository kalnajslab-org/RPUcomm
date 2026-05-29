/*
 *  RPUcomm.h
 *  Derived from PUcomm.h
 *  
 *  This file declares an Arduino library (C++ class) that implements the communication
 *  between the RATCHuTS and the RPU. The class inherits its protocol from the SerialComm
 *  class.
 */

#ifndef RPUcomm_H
#define RPUcomm_H

#include "SerialComm.h"

enum RPUMessages_t : uint8_t {
    RPU_NO_MESSAGE = 0,

    // RATCHuTS -> RPU (no params)
    RPU_SEND_STATUS, //1
    RPU_SEND_PROFILE_RECORD, //2
    RPU_SEND_TSEN_RECORD, //3
    RPU_RESET, //4

    // RATCHuTS -> RPU (with params)
    RPU_SET_HEATERS, //5
    RPU_GO_LOWPOWER, //6
    RPU_GO_IDLE,  //7
    RPU_GO_WARMUP, //8
    RPU_GO_PREPROFILE, //9
    RPU_GO_PROFILE, //10
    RPU_UPDATE_GPS, //11
    RPU_LORA_STATUS,
    RPU_LORA_TM,

    // RPU -> RATCHuTS (no params)
    RPU_IS_DOCKED, //12
    RPU_NO_MORE_RECORDS, //13
    RPU_PROFILE_RECORD,  // 14 binary transfer
    RPU_TSEN_RECORD, // 15 binary transfer

    // RPU -> RATCHuTS (with params)
    RPU_STATUS, //16
    RPU_ERROR //17
};


class RPUcomm : public SerialComm {
public:
    RPUcomm(Stream * serial_port);
    ~RPUcomm() { };

    // RATCHuTS -> RPU (with params) -----------------------
    bool TX_SetHeaters(float Heater1T, float Heater2T); //Set the heater temperature (parameter not state)
    bool RX_SetHeaters(float * Heater1T, float * Heater2T);

    bool TX_LowPower(float survivalT); //go low power, heater set to survival_temp
    bool RX_LowPower(float * survivalT);

    bool TX_Idle(int32_t TSENTMRate); //idle, TSEN TM packet ready every TMRate seconds
    bool RX_Idle(int32_t * TSENTMRate); //idle, TSEN TM packet ready every TMRate seconds

    bool TX_WarmUp(float FLASH_T, float Heater_1_T, float Heater_2_T, int8_t FLASH_power, int8_t TSEN_power);
    bool RX_WarmUp(float * FLASH_T, float * Heater_1_T, float * Heater_2_T, int8_t * FLASH_power, int8_t * TSEN_power);

    bool TX_PreProfile(int32_t preTime, int32_t TM_period, int32_t data_rate, int8_t TSEN_power, int8_t ROPC_power, int8_t FLASH_power);
    bool RX_PreProfile(int32_t * preTime, int32_t * TM_period, int32_t * data_rate, int8_t * TSEN_power, int8_t * ROPC_power, int8_t * FLASH_power);

    bool TX_Profile(int32_t t_down, int32_t t_dwell, int32_t t_up, int32_t rate_profile, int32_t rate_dwell, int8_t TSEN_power, int8_t ROPC_power, int8_t FLASH_power, int8_t LoRa_TM);
    bool RX_Profile(int32_t * t_down, int32_t * t_dwell, int32_t * t_up, int32_t * rate_profile, int32_t * rate_dwell, int8_t * TSEN_power, int8_t * ROPC_power, int8_t * FLASH_power, int8_t * LoRa_TM);

    bool TX_UpdateGPS(uint32_t ZephyrGPSTime, float ZephyrGPSlat, float ZephyrGPSlon, uint16_t ZephyrGPSAlt);
    bool RX_UpdateGPS(uint32_t * ZephyrGPSTime, float * ZephyrGPSlat, float * ZephyrGPSlon, uint16_t * ZephyrGPSAlt);

    bool TX_RPULoRaStatus(uint16_t LoRaTXStatus);
    bool RX_RPULoRaStatus(uint16_t * LoRaTXStatus);

    bool TX_RPULoRaTM(uint8_t LoRaTXTM);
    bool RX_RPULoRaTM(uint8_t * LoRaTXTM);

    // RPU -> RATCHuTS (with params) -----------------------

    bool TX_Status(uint32_t RPUTime, float VBattery, float ICharge, float Therm1T, float Therm2T, uint8_t HeaterStat);
    bool RX_Status(uint32_t * RPUTime, float * VBattery, float * ICharge, float * Therm1T, float * Therm2T, uint8_t * HeaterStat);

    bool TX_Error(const char * error);
    bool RX_Error(char * error, uint8_t buffer_size);
};

#endif /* RPUcomm_H */
