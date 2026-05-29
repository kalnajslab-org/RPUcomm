#include <Arduino.h>
#include "RPUcomm.h"

RPUcomm rpu(&Serial1);

void setup() {
    float f1, f2, f3;
    int8_t i1, i2, i3, i4, i5, i6;
    int32_t l1, l2, l3, l4, l5;
    uint8_t u1;
    uint16_t u2;
    uint32_t u3;
    char buf[64];

    rpu.TX_SetHeaters(0.0f, 0.0f);
    rpu.RX_SetHeaters(&f1, &f2);

    rpu.TX_LowPower(0.0f);
    rpu.RX_LowPower(&f1);

    rpu.TX_Idle(0);
    rpu.RX_Idle(&l1);

    rpu.TX_WarmUp(0.0f, 0.0f, 0.0f, 0, 0);
    rpu.RX_WarmUp(&f1, &f2, &f3, &i1, &i2);

    rpu.TX_PreProfile(0, 0, 0, 0, 0, 0);
    rpu.RX_PreProfile(&l1, &l2, &l3, &i1, &i2, &i3);

    rpu.TX_Profile(0, 0, 0, 0, 0, 0, 0, 0, 0);
    rpu.RX_Profile(&l1, &l2, &l3, &l4, &l5, &i1, &i2, &i3, &i4);

    rpu.TX_UpdateGPS(0, 0.0f, 0.0f, 0);
    rpu.RX_UpdateGPS(&u3, &f1, &f2, &u2);

    rpu.TX_RPULoRaStatus(0);
    rpu.RX_RPULoRaStatus(&u2);

    rpu.TX_RPULoRaTM(0);
    rpu.RX_RPULoRaTM(&u1);

    rpu.TX_Status(0, 0.0f, 0.0f, 0.0f, 0.0f, 0);
    rpu.RX_Status(&u3, &f1, &f2, &f3, &f3, &u1);

    rpu.TX_Error("test");
    rpu.RX_Error(buf, sizeof(buf));
}

void loop() {}
