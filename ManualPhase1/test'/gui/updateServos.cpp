#include <Arduino.h>
#include "updateServos.h"

void updateServos(int prev[], int curr[], float slowness) {
    for (int i = 0; i < NUM_SERVOS; i++) {

        if (prev[i] == curr[i]) continue;

        if (prev[i] < curr[i]) {
            for (int a = prev[i]; a <= curr[i]; a++) {
                s[i].write(a);
                delay(slowness);
            }
        } else {
            for (int a = prev[i]; a >= curr[i]; a--) {
                s[i].write(a);
                delay(slowness);
            }
        }
    }
}
