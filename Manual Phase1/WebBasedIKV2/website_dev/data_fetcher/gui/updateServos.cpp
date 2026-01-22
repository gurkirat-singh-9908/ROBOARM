#include "updateServos.h"

void updateServos(int prevAngles[], int crntAngles[], float slowness) {
  for (int i = 0; i < 6; i++) {
    Write(i, prevAngles[i], crntAngles[i], slowness);
    prevAngles[i] = crntAngles[i];
  }
}

void Write(int sno, int prevA, int crntA, float slowness) {
  if (prevA < crntA) {
    for (int i = prevA; i <= crntA; i++) {
      delay(slowness);
      Serial.print("sno ");
      Serial.println(sno + 1);
      Serial.print("write ");
      Serial.println(i);
      switch (sno) {
        case 0: s1.write(i); break;
        case 1: s2.write(i); break;
        case 2: s3.write(i); break;
        case 3: s4.write(i); break;
        case 4: s5.write(i); break;
        case 5: s6.write(i); break;
      }
    }
  } else {
    for (int i = prevA; i >= crntA; i--) {
      delay(slowness);
      Serial.print("sno ");
      Serial.println(sno + 1);
      Serial.print("write ");
      Serial.println(i);
      switch (sno) {
        case 0: s1.write(i); break;
        case 1: s2.write(i); break;
        case 2: s3.write(i); break;
        case 3: s4.write(i); break;
        case 4: s5.write(i); break;
        case 5: s6.write(i); break;
      }
    }
  }
}