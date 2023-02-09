package org.firstinspires.ftc.teamcode.subsystems

enum class RobotConfig {
    LEFT_FRONT, LEFT_BACK, RIGHT_FRONT, RIGHT_BACK, IMU, LOW_LIFT, TOP_LIFT, CLAW, WEBCAM_1, WEBCAM_2, RETRACTOR;

    @JvmField val s: String = this.name.replace("_", "").lowercase()
}
