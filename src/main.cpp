#include <Arduino.h>
#include "RPUComm.h"

RPUComm rpu(&Serial1);

void setup() {
    int8_t  i1, i2, i3, i4;
    int32_t l1, l2;
    float f1;
    char buf[64];

    rpu.TX_GoMeasure(0, 0, 0.0f, 0, 0, 0, 0);
    rpu.RX_GoMeasure(&l1, &l2, &f1, &i1, &i2, &i3, &i4);

    rpu.TX_Status("{}");
    rpu.RX_Status(buf, sizeof(buf));

    rpu.TX_Error("test");
    rpu.RX_Error(buf, sizeof(buf));
}

void loop() {}
