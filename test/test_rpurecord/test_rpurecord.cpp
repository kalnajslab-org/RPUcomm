// To run these tests in VSCode, in the PlatformIO pane go to:
// PROJECT TASKS -> General -> Advanced -> Test.

#include <Arduino.h>
#include <unity.h>
#include "RPUComm.h"

void setUp(void) {}
void tearDown(void) {}

void test_record_size_constant() {
    TEST_ASSERT_EQUAL_UINT32(38, RPU_RECORD_BYTES);
}

void test_buffer_too_small() {
    RPURecord rec;
    uint8_t buf[RPU_RECORD_BYTES - 1];
    TEST_ASSERT_FALSE(rec.encode(buf, sizeof(buf)));
    TEST_ASSERT_FALSE(rec.decode(buf, sizeof(buf)));
}

void test_fast_field_roundtrip() {
    RPURecord rec;
    rec.setElapsedS(12345);
    rec.setAlt(1234);
    rec.setLatDelta(0.12345);
    rec.setLonDelta(-0.5);
    rec.setSats(7);
    rec.setGpsAge(10);
    rec.setOpcD300(1000);
    rec.setOpcD2000(2000);
    rec.setTsenAirt(0xABC);
    rec.setTsenPres(0x123456);
    rec.setTsenPtemp(0xFEDCBA);
    rec.setRs41AirT(23.45f);
    rec.setRs41Pres(987.6f);
    rec.setRs41Humidity(55.55f);
    rec.setRs41HSensorT(10.0f);
    rec.setTdlasMrAvg(50.0f);
    rec.setTdlasBkg(20.0f);
    rec.setTdlasPeak(12.0f);
    rec.setTdlasRatio(0.5f);

    uint8_t buf[RPU_RECORD_BYTES];
    TEST_ASSERT_TRUE(rec.encode(buf, sizeof(buf)));

    RPURecord decoded;
    TEST_ASSERT_TRUE(decoded.decode(buf, sizeof(buf)));

    TEST_ASSERT_EQUAL_UINT32(12345, decoded.getElapsedS());
    TEST_ASSERT_EQUAL_FLOAT(1234.0f, decoded.getAlt());
    TEST_ASSERT_FLOAT_WITHIN(0.00002f, 0.12345f, decoded.getLatDelta());
    TEST_ASSERT_FLOAT_WITHIN(0.00002f, -0.5f, decoded.getLonDelta());
    TEST_ASSERT_EQUAL_UINT8(7, decoded.getSats());
    TEST_ASSERT_EQUAL_UINT32(10, decoded.getGpsAge());
    TEST_ASSERT_EQUAL_UINT16(1000, decoded.getOpcD300());
    TEST_ASSERT_EQUAL_UINT16(2000, decoded.getOpcD2000());
    TEST_ASSERT_EQUAL_UINT16(0xABC, decoded.getTsenAirt());
    TEST_ASSERT_EQUAL_UINT16(0x1234, decoded.getTsenPres());
    TEST_ASSERT_EQUAL_UINT16(0xFEDC, decoded.getTsenPtemp());
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 23.45f, decoded.getRs41AirT());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 987.6f, decoded.getRs41Pres());
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 55.55f, decoded.getRs41Humidity());
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 10.0f, decoded.getRs41HSensorT());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 50.0f, decoded.getTdlasMrAvg());
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 20.0f, decoded.getTdlasBkg());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 12.0f, decoded.getTdlasPeak());
    TEST_ASSERT_FLOAT_WITHIN(0.0005f, 0.5f, decoded.getTdlasRatio());
}

// Sets every slow/round-robin field to a fixed test value, regardless of which
// slot is active, mirroring how RPU.cpp calls all slow setters every tick.
void set_all_slow_fields(RPURecord &rec) {
    rec.setOpcD500(100);
    rec.setOpcD700(200);
    rec.setOpcD1000(300);
    rec.setOpcD2500(400);
    rec.setOpcD3000(500);
    rec.setOpcD5000(600);
    rec.setRs41Hdg(123.0f);
    rec.setBemfV(3.3f);
    rec.setTdlasSpec1(1111);
    rec.setTdlasSpec2(2222);
    rec.setTdlasSpec3(3333);
    rec.setTdlasSpec4(4444);
    rec.setTsenI(40);
    rec.setOpcI(80);
    rec.setPumpI(120);
    rec.setTdlasI(160);
    rec.setV5V(3.0f);
    rec.setBatT(20);
    rec.setPumpT(25);
    rec.setPcbT(30);
    rec.setBatV(12.34f);
    rec.setHeaterStat(5);
}

// Verifies that decode() only populates the slow fields belonging to the
// given slot, leaving all other slow fields at their default (zero) value.
void check_slot_fields(uint8_t slot, const RPURecord &decoded) {
    TEST_ASSERT_EQUAL_UINT16(slot == 0 ? 100 : 0, decoded.getOpcD500());
    TEST_ASSERT_EQUAL_UINT16(slot == 0 ? 200 : 0, decoded.getOpcD700());
    TEST_ASSERT_EQUAL_UINT16(slot == 1 ? 300 : 0, decoded.getOpcD1000());
    TEST_ASSERT_EQUAL_UINT16(slot == 1 ? 400 : 0, decoded.getOpcD2500());
    TEST_ASSERT_EQUAL_UINT16(slot == 2 ? 500 : 0, decoded.getOpcD3000());
    TEST_ASSERT_EQUAL_UINT16(slot == 2 ? 600 : 0, decoded.getOpcD5000());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, slot == 3 ? 123.0f : 0.0f, decoded.getRs41Hdg());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, slot == 3 ? 3.3f : 0.0f, decoded.getBemfV());
    TEST_ASSERT_EQUAL_FLOAT(slot == 4 ? 1111.0f : 0.0f, decoded.getTdlasSpec1());
    TEST_ASSERT_EQUAL_FLOAT(slot == 4 ? 2222.0f : 0.0f, decoded.getTdlasSpec2());
    TEST_ASSERT_EQUAL_FLOAT(slot == 5 ? 3333.0f : 0.0f, decoded.getTdlasSpec3());
    TEST_ASSERT_EQUAL_FLOAT(slot == 5 ? 4444.0f : 0.0f, decoded.getTdlasSpec4());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, slot == 6 ? 40.0f : 0.0f, decoded.getTsenI());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, slot == 6 ? 80.0f : 0.0f, decoded.getOpcI());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, slot == 6 ? 120.0f : 0.0f, decoded.getPumpI());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, slot == 6 ? 160.0f : 0.0f, decoded.getTdlasI());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, slot == 6 ? 3.0f : 0.0f, decoded.getV5V());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, slot == 7 ? 20.0f : -100.0f, decoded.getBatT());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, slot == 7 ? 25.0f : -100.0f, decoded.getPumpT());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, slot == 7 ? 30.0f : -100.0f, decoded.getPcbT());
    TEST_ASSERT_FLOAT_WITHIN(0.005f, slot == 7 ? 12.34f : 0.0f, decoded.getBatV());
    TEST_ASSERT_EQUAL_UINT8(slot == 7 ? 5 : 0, decoded.getHeaterStat());
}

void test_round_robin_slots() {
    RPURecord::resetRotation();

    // 9 iterations: one per slot (0-7), plus one extra to confirm the
    // rotation wraps back around to slot 0 after slot 7.
    for (uint8_t i = 0; i < 9; ++i) {
        uint8_t slot = i % 8;

        RPURecord rec;
        set_all_slow_fields(rec);

        uint8_t buf[RPU_RECORD_BYTES];
        TEST_ASSERT_TRUE(rec.encode(buf, sizeof(buf)));

        RPURecord decoded;
        TEST_ASSERT_TRUE(decoded.decode(buf, sizeof(buf)));

        check_slot_fields(slot, decoded);

        RPURecord::advanceRotation();
    }
}

void test_setter_clamping() {
    RPURecord rec;
    rec.setSats(20);        // clamps to 15 (4 bits)
    rec.setGpsAge(100);      // clamps to 15 (4 bits)
    rec.setLatDelta(10.0);   // clamps to int16 max -> 32767 / 50000

    uint8_t buf[RPU_RECORD_BYTES];
    TEST_ASSERT_TRUE(rec.encode(buf, sizeof(buf)));

    RPURecord decoded;
    TEST_ASSERT_TRUE(decoded.decode(buf, sizeof(buf)));

    TEST_ASSERT_EQUAL_UINT8(15, decoded.getSats());
    TEST_ASSERT_EQUAL_UINT32(15, decoded.getGpsAge());
    TEST_ASSERT_FLOAT_WITHIN(0.00002f, 32767.0f / 50000.0f, decoded.getLatDelta());
}

void setup() {
    delay(2000);

    UNITY_BEGIN();
    RUN_TEST(test_record_size_constant);
    RUN_TEST(test_buffer_too_small);
    RUN_TEST(test_fast_field_roundtrip);
    RUN_TEST(test_round_robin_slots);
    RUN_TEST(test_setter_clamping);
    UNITY_END();
}

void loop() {}
