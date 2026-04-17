#include "updateServos.h"

void updateServos(int crntAngles[]) {
  for (int i = 0; i < 6; i++) {
    Write(i, crntAngles[i]);
  }
}

void Write(int sno, int crntA) {
    switch (sno) {
      case 0: s1.write(crntA); break;
      case 1: s2.write(crntA); break;
      case 2: s3.write(crntA); break;
      case 3: s4.write(crntA); break;
      case 4: s5.write(crntA); break;
      case 5: s6.write(crntA); break;
  }
}
