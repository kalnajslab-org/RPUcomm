/*
 *  RPUComm.h
 *  Derived from PUcomm.h
 *  
 *  This file declares an Arduino library (C++ class) that implements the communication
 *  between the RATCHuTS and the RPU. The class inherits its protocol from the SerialComm
 *  class.
 */

#ifndef RPUComm_H
#define RPUComm_H

#include "SerialComm.h"

enum RPUMessages_t : uint8_t {
    RPU_NO_MESSAGE = 0,        // —
    RPU_SEND_STATUS,           // RATCHUTS→RPU
    RPU_SEND_RECORDS,          // RATCHUTS→RPU
    RPU_GO_MEASURE,            // RATCHUTS→RPU | duration(int32_t s), rate(int32_t s), opc(int8_t), tdlas(int8_t), tsen(int8_t)
    RPU_GO_STANDBY,            // RATCHUTS→RPU
    RPU_SET_BATT_T,            // RATCHUTS→RPU | setpoint(float °C)
    RPU_SET_V_LOW_BATT,        // RATCHUTS→RPU | threshold(float V)
    RPU_SET_V_CRIT_BATT,       // RATCHUTS→RPU | threshold(float V)
    RPU_SET_STATUS_RATE,       // RATCHUTS→RPU | interval(uint32_t s)
    RPU_NO_MORE_RECORDS,       // RPU→RATCHUTS
    RPU_STATUS,                // RPU→RATCHUTS | time(uint32_t ms), vbat(float V), icharge(float A), therm1(float °C), therm2(float °C), heater(uint8_t)
    RPU_ERROR                  // RPU→RATCHUTS | message(string)
};


class RPUComm : public SerialComm {
public:
    RPUComm(Stream * serial_port);
    ~RPUComm() { };

    // RATCHuTS -> RPU (with params) -----------------------
    bool TX_GoMeasure(int32_t duration, int32_t rate, int8_t opc_power, int8_t tdlas_power, int8_t tsen_power);
    bool RX_GoMeasure(int32_t * duration, int32_t * rate, int8_t * opc_power, int8_t * tdlas_power, int8_t * tsen_power);


    // RPU -> RATCHuTS (with params) -----------------------

    bool TX_Status(uint32_t RPUTime, float VBattery, float ICharge, float Therm1T, float Therm2T, uint8_t HeaterStat);
    bool RX_Status(uint32_t * RPUTime, float * VBattery, float * ICharge, float * Therm1T, float * Therm2T, uint8_t * HeaterStat);

    bool TX_Error(const char * error);
    bool RX_Error(char * error, uint8_t buffer_size);
};

#endif /* RPUComm_H */
