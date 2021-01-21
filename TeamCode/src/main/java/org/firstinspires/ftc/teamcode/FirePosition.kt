package org.firstinspires.ftc.teamcode

enum class FirePosition {
    HIGH, MEDIUM, LOW;

    override fun toString(): String {
        if (this == HIGH) {
            return "Fire position: high"
        }
        if (this == MEDIUM) {
            return "Fire position: medium"
        }
        return if (this == LOW) {
            "Fire position: low"
        } else "Fire position: unknown"
    }
}