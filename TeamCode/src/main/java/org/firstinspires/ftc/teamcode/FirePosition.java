package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

@SuppressWarnings("unused")
public enum FirePosition {
    HIGH,
    MEDIUM,
    LOW,
    ;

    @NonNull
    @Override
    public String toString() {
        if (this == FirePosition.HIGH) {

            return "Fire position: high";
        }
        if (this == FirePosition.MEDIUM) {
            return "Fire position: medium";
        }
        if (this == FirePosition.LOW) {
            return "Fire position: low";
        }
        return "Fire position: unknown";
    }
}
