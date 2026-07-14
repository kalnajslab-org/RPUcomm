#include <Arduino.h>
#include "RPUComm.h"

RPUComm rpu(&Serial1);

void setup() {
    Serial.begin(115200);
    while (!Serial);

    delay(2000); // give time for the serial monitor to open

    Serial.println("RPUComm test");

    Serial1.begin(115200);
    // Serial1 is not connected to anything, so RX calls will return garbage.
    // Connect a loopback (TX->RX) or an actual RPU for meaningful output.
}

void loop() {

    int8_t  i1, i2, i3, i4;
    int32_t l1, l2;
    float f1;
    char buf[64];
    uint32_t count = 0;

    while (1) {
        Serial.printf("--- iteration %lu ---\n", count++);

        rpu.TX_GoMeasure(0, 0, 0.0f, 0, 0, 0, 0);
        rpu.RX_GoMeasure(&l1, &l2, &f1, &i1, &i2, &i3, &i4);

        rpu.TX_Status("{}");
        rpu.RX_Status(buf, sizeof(buf));

        rpu.TX_Error("test");
        rpu.RX_Error(buf, sizeof(buf));

        Serial.printf("GoMeasure: %ld, %ld, %f, %d, %d, %d, %d\n", l1, l2, f1, i1, i2, i3, i4);
        Serial.printf("Status: %s\n", buf);
        Serial.printf("Error: %s\n", buf);

        delay(1000);
    }
}
