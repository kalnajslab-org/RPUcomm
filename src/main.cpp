#include <Arduino.h>
#include "RPUComm.h"

RPUComm rpu(&Serial1);

void setup() {
    int8_t  i1, i2, i3;
    int32_t l1, l2;
    uint8_t u1;
    uint32_t u3;
    float f1, f2, f3, f4, f5;
    char buf[64];

    rpu.TX_GoMeasure(0, 0, 0, 0, 0);
    rpu.RX_GoMeasure(&l1, &l2, &i1, &i2, &i3);

    rpu.TX_Status(0, 0.0f, 0.0f, 0.0f, 0.0f, 0);
    rpu.RX_Status(&u3, &f1, &f2, &f3, &f4, &u1);

    rpu.TX_Error("test");
    rpu.RX_Error(buf, sizeof(buf));
}

void loop() {}
