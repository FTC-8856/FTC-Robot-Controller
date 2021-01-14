package org.firstinspires.ftc.teamcode;

@SuppressWarnings("unused")
public enum FirePosition {
    HIGH,
    MEDIUM,
    LOW,
    ;

    @Override
    public String toString() {
        String level = "unknown";
        switch (this) {
            case HIGH:
                level = "high";
                break;
            case MEDIUM:
                level = "medium";
                break;
            case LOW:
                level = "low";
                break;
            default:
                break;
        }
        return level;
    }

    public double toServoPosition() {
        double level = 0.0;
        switch (this) {
            case HIGH:
                level = 1.0;
                break;
            case MEDIUM:
                level = 0.5;
                break;
            case LOW:
                level = 0.2;
                break;
            default:
                break;
        }
        return level;
    }
}
