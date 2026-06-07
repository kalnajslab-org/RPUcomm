/*
 *  RPUComm.cpp
 *  Derived from PUcomm.cpp
 *  
 *  This file implements an Arduino library (C++ class) that implements the communication
 *  between the RATCHuTS and RPU. The class inherits its protocol from the SerialComm
 *  class.
 */

#include "RPUComm.h"
#include <etl/bit_stream.h>

RPUComm::RPUComm(Stream * serial_port)
    : SerialComm(serial_port)
{
}

// RATCHuTS -> RPU (with params) ---------------------------

// RATCHuTS -> RPU: 2026 commands -------------------------

bool RPUComm::TX_GoMeasure(int32_t duration, int32_t rate, int8_t opc_power, int8_t tdlas_power, int8_t tsen_power)
{
    if (!Add_int32(duration))    return false;
    if (!Add_int32(rate))        return false;
    if (!Add_int8(opc_power))    return false;
    if (!Add_int8(tdlas_power))  return false;
    if (!Add_int8(tsen_power))   return false;
    TX_ASCII(RPU_GO_MEASURE);
    return true;
}

bool RPUComm::RX_GoMeasure(int32_t * duration, int32_t * rate, int8_t * opc_power, int8_t * tdlas_power, int8_t * tsen_power)
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

bool RPUComm::TX_Status(uint32_t RPUTime, float VBattery, float ICharge, float Therm1T, float Therm2T, uint8_t HeaterStat)
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

bool RPUComm::RX_Status(uint32_t * RPUTime, float * VBattery, float * ICharge, float * Therm1T, float * Therm2T, uint8_t * HeaterStat)
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

bool RPUComm::TX_Error(const char * error)
{
    //if (Add_string(error)) return false;

    TX_String(RPU_ERROR,error);

    return true;
}

bool RPUComm::RX_Error(char * error, uint8_t buffer_size)
{
    return Get_string(error, buffer_size);
}

// ---------------------------------------------------------------------------
// RPUPacket
// ---------------------------------------------------------------------------

void RPUPacket::setBoardId(uint16_t id)        { board_id_    = id; }
void RPUPacket::setState(uint8_t state)        { state_       = state; }
void RPUPacket::setWdtCount(uint8_t count)     { wdt_count_   = count; }
void RPUPacket::setVin(float volts)            { vin_raw_     = (uint8_t)constrain((int)(volts * 10.0f), 0, 255); }
void RPUPacket::setV5V(float volts)            { v5v_raw_     = (uint8_t)constrain((int)(volts * 10.0f), 0, 255); }
void RPUPacket::setBatV(float volts)           { bat_v_raw_   = (uint8_t)constrain((int)(volts * 10.0f), 0, 255); }
void RPUPacket::setHeaterDuty(uint8_t percent) { heater_duty_ = (uint8_t)constrain((int)percent, 0, 100); }
void RPUPacket::setChgI(float amps)            { chg_i_raw_   = (uint8_t)constrain((int)(amps * 20.0f), 0, 127); }
void RPUPacket::setBatT(float celsius)         { bat_t_raw_   = (uint16_t)constrain((int)((celsius + 100.0f) * 2.0f), 0, 511); }
void RPUPacket::setPcbT(float celsius)         { pcb_t_raw_   = (uint16_t)constrain((int)((celsius + 100.0f) * 2.0f), 0, 511); }
void RPUPacket::setPumpI(float milliamps)      { pump_i_raw_   = (uint16_t)constrain((int)milliamps, 0, 4095); }
void RPUPacket::setOpcI(float milliamps)       { opc_i_raw_    = (uint16_t)constrain((int)milliamps, 0, 4095); }
void RPUPacket::setTsenI(float milliamps)      { tsen_i_raw_   = (uint16_t)constrain((int)milliamps, 0, 4095); }
void RPUPacket::setTdlasI(float milliamps)     { tdlas_i_raw_  = (uint16_t)constrain((int)milliamps, 0, 4095); }
void RPUPacket::setHeaterI(float milliamps)    { heater_i_raw_ = (uint16_t)constrain((int)milliamps, 0, 4095); }
void RPUPacket::setLat(double degrees)         { lat_raw_      = (uint32_t)constrain((int)((degrees + 90.0)  * 10000.0), 0, 1800000); }
void RPUPacket::setLon(double degrees)         { lon_raw_      = (uint32_t)constrain((int)((degrees + 180.0) * 10000.0), 0, 3600000); }
void RPUPacket::setAlt(float meters)           { alt_raw_      = (uint16_t)constrain((int)meters, 0, 65535); }
void RPUPacket::setSats(uint8_t count)         { sats_         = (uint8_t)constrain((int)count, 0, 31); }

void RPUPacket::setVer(const char * ver)
{
    memset(ver_, 0, sizeof(ver_));
    if (ver != nullptr) {
        strncpy(ver_, ver, sizeof(ver_) - 1);
    }
}

bool RPUPacket::encode(uint8_t * buf, size_t buf_size) const
{
    if (buf_size < RPU_PKT_BYTES) return false;

    memset(buf, 0, RPU_PKT_BYTES);
    etl::bit_stream_writer bsw(buf, RPU_PKT_BYTES, etl::endian::big);

    bsw.write_unchecked<uint8_t> (RPU_PKT_VERSION,  RPU_PKT_VER_BITS);
    bsw.write_unchecked<uint16_t>(board_id_,        RPU_PKT_ID_BITS);
    bsw.write_unchecked<uint8_t> (state_,           RPU_PKT_STATE_BITS);
    bsw.write_unchecked<uint8_t> (wdt_count_,       RPU_PKT_WDT_BITS);
    bsw.write_unchecked<uint8_t> (vin_raw_,         RPU_PKT_VIN_BITS);
    bsw.write_unchecked<uint8_t> (v5v_raw_,         RPU_PKT_V5_BITS);
    bsw.write_unchecked<uint8_t> (bat_v_raw_,       RPU_PKT_BATV_BITS);
    bsw.write_unchecked<uint8_t> (heater_duty_,     RPU_PKT_DUTY_BITS);
    bsw.write_unchecked<uint8_t> (chg_i_raw_,       RPU_PKT_CHGI_BITS);
    bsw.write_unchecked<uint16_t>(bat_t_raw_,       RPU_PKT_TEMP_BITS);
    bsw.write_unchecked<uint16_t>(pcb_t_raw_,       RPU_PKT_TEMP_BITS);
    bsw.write_unchecked<uint16_t>(pump_i_raw_,      RPU_PKT_CURR_BITS);
    bsw.write_unchecked<uint16_t>(opc_i_raw_,       RPU_PKT_CURR_BITS);
    bsw.write_unchecked<uint16_t>(tsen_i_raw_,      RPU_PKT_CURR_BITS);
    bsw.write_unchecked<uint16_t>(tdlas_i_raw_,     RPU_PKT_CURR_BITS);
    bsw.write_unchecked<uint16_t>(heater_i_raw_,    RPU_PKT_CURR_BITS);
    bsw.write_unchecked<uint32_t>(lat_raw_,         RPU_PKT_LAT_BITS);
    bsw.write_unchecked<uint32_t>(lon_raw_,         RPU_PKT_LON_BITS);
    bsw.write_unchecked<uint16_t>(alt_raw_,         RPU_PKT_ALT_BITS);
    bsw.write_unchecked<uint8_t> (sats_,            RPU_PKT_SATS_BITS);

    for (size_t i = 0; i < RPU_PKT_FW_VER_LEN; ++i) {
        bsw.write_unchecked<uint8_t>((uint8_t)ver_[i], 8);
    }

    return true;
}

bool RPUPacket::decode(const uint8_t * buf, size_t buf_size)
{
    if (buf_size < RPU_PKT_BYTES) return false;

    etl::bit_stream_reader bsr(buf, RPU_PKT_BYTES, etl::endian::big);

    bsr.read_unchecked<uint8_t>(RPU_PKT_VER_BITS);  // packet version, not stored

    board_id_     = bsr.read_unchecked<uint16_t>(RPU_PKT_ID_BITS);
    state_        = bsr.read_unchecked<uint8_t> (RPU_PKT_STATE_BITS);
    wdt_count_    = bsr.read_unchecked<uint8_t> (RPU_PKT_WDT_BITS);
    vin_raw_      = bsr.read_unchecked<uint8_t> (RPU_PKT_VIN_BITS);
    v5v_raw_      = bsr.read_unchecked<uint8_t> (RPU_PKT_V5_BITS);
    bat_v_raw_    = bsr.read_unchecked<uint8_t> (RPU_PKT_BATV_BITS);
    heater_duty_  = bsr.read_unchecked<uint8_t> (RPU_PKT_DUTY_BITS);
    chg_i_raw_    = bsr.read_unchecked<uint8_t> (RPU_PKT_CHGI_BITS);
    bat_t_raw_    = bsr.read_unchecked<uint16_t>(RPU_PKT_TEMP_BITS);
    pcb_t_raw_    = bsr.read_unchecked<uint16_t>(RPU_PKT_TEMP_BITS);
    pump_i_raw_   = bsr.read_unchecked<uint16_t>(RPU_PKT_CURR_BITS);
    opc_i_raw_    = bsr.read_unchecked<uint16_t>(RPU_PKT_CURR_BITS);
    tsen_i_raw_   = bsr.read_unchecked<uint16_t>(RPU_PKT_CURR_BITS);
    tdlas_i_raw_  = bsr.read_unchecked<uint16_t>(RPU_PKT_CURR_BITS);
    heater_i_raw_ = bsr.read_unchecked<uint16_t>(RPU_PKT_CURR_BITS);
    lat_raw_      = bsr.read_unchecked<uint32_t>(RPU_PKT_LAT_BITS);
    lon_raw_      = bsr.read_unchecked<uint32_t>(RPU_PKT_LON_BITS);
    alt_raw_      = bsr.read_unchecked<uint16_t>(RPU_PKT_ALT_BITS);
    sats_         = bsr.read_unchecked<uint8_t> (RPU_PKT_SATS_BITS);

    for (size_t i = 0; i < RPU_PKT_FW_VER_LEN; ++i) {
        ver_[i] = (char)bsr.read_unchecked<uint8_t>(8);
    }
    ver_[RPU_PKT_FW_VER_LEN - 1] = '\0';

    return true;
}

int RPUPacket::toJSON(char * buf, size_t buf_size) const
{
    static const char * state_names[] = { "STANDBY", "MEASURE", "ERROR" };
    const char * state_str = (state_ < (sizeof(state_names) / sizeof(state_names[0])))
                             ? state_names[state_]
                             : "UNKNOWN";

    return snprintf(buf, buf_size,
        "{\"id\":\"%04X\",\"ver\":\"%s\",\"state\":\"%s\",\"wdt_n\":%u,"
        "\"vin\":%.1f,\"v5\":%.1f,\"bat_v\":%.1f,\"bat_duty\":%u,\"chg_i\":%.1f,"
        "\"bat_t\":%.1f,\"pcb_t\":%.1f,"
        "\"pump_i\":%.0f,\"opc_i\":%.0f,\"tsen_i\":%.0f,\"tdlas_i\":%.0f,\"heater_i\":%.0f,"
        "\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,\"sats\":%u}",
        board_id_, ver_, state_str, wdt_count_,
        getVin(), getV5V(), getBatV(), heater_duty_, getChgI(),
        getBatT(), getPcbT(),
        getPumpI(), getOpcI(), getTsenI(), getTdlasI(), getHeaterI(),
        getLat(), getLon(), getAlt(), sats_);
}


