/*
 *  RPUcomm.cpp
 *  Derived from PUcomm.cpp
 *  
 *  This file implements an Arduino library (C++ class) that implements the communication
 *  between the RATCHuTS and RPU. The class inherits its protocol from the SerialComm
 *  class.
 */

#include "RPUcomm.h"

RPUcomm::RPUcomm(Stream * serial_port)
    : SerialComm(serial_port)
{
}

// RATCHuTS -> RPU (with params) ---------------------------

// RATCHuTS -> RPU: 2026 commands -------------------------

bool RPUcomm::TX_GoMeasure(int32_t duration, int32_t rate, int8_t opc_power, int8_t tdlas_power, int8_t tsen_power)
{
    if (!Add_int32(duration))    return false;
    if (!Add_int32(rate))        return false;
    if (!Add_int8(opc_power))    return false;
    if (!Add_int8(tdlas_power))  return false;
    if (!Add_int8(tsen_power))   return false;
    TX_ASCII(RPU_GO_MEASURE);
    return true;
}

bool RPUcomm::RX_GoMeasure(int32_t * duration, int32_t * rate, int8_t * opc_power, int8_t * tdlas_power, int8_t * tsen_power)
{
    int32_t t, r;
    int8_t  o, d, s;
    if (!Get_int32(&t)) return false;
    if (!Get_int32(&r)) return false;
    if (!Get_int8(&o))  return false;
    if (!Get_int8(&d))  return false;
    if (!Get_int8(&s))  return false;
    *duration   = t;
    *rate       = r;
    *opc_power  = o;
    *tdlas_power = d;
    *tsen_power = s;
    return true;
}

bool RPUcomm::TX_Status(uint32_t RPUTime, float VBattery, float ICharge, float Therm1T, float Therm2T, uint8_t HeaterStat)
{
    if (!Add_uint32(RPUTime)) return false;
    if (!Add_float(VBattery)) return false;
    if (!Add_float(ICharge)) return false;
    if (!Add_float(Therm1T)) return false;
    if (!Add_float(Therm2T)) return false;
    if (!Add_uint8(HeaterStat)) return false;
   
    TX_ASCII(RPU_STATUS);

    return true;
}

bool RPUcomm::RX_Status(uint32_t * RPUTime, float * VBattery, float * ICharge, float * Therm1T, float * Therm2T, uint8_t * HeaterStat)
{
    uint32_t temp1;
    float temp2, temp3, temp4, temp5;
    uint8_t temp6;

    if (!Get_uint32(&temp1)) return false;
    if (!Get_float(&temp2)) return false;
    if (!Get_float(&temp3)) return false;
    if (!Get_float(&temp4)) return false;
    if (!Get_float(&temp5)) return false;
    if (!Get_uint8(&temp6)) return false;


    *RPUTime = temp1;
    *VBattery = temp2;
    *ICharge = temp3;
    *Therm1T = temp4;
    *Therm2T = temp5;
    *HeaterStat = temp6;

    return true;
}

// -- RPU to RATCHuTS error string

bool RPUcomm::TX_Error(const char * error)
{
    //if (Add_string(error)) return false;

    TX_String(RPU_ERROR,error);

    return true;
}

bool RPUcomm::RX_Error(char * error, uint8_t buffer_size)
{
    return Get_string(error, buffer_size);
}


