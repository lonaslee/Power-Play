package org.firstinspires.ftc.teamcode.robot

enum class Config {
    LEFT_FRONT, LEFT_BACK, RIGHT_FRONT, RIGHT_BACK, IMU, LOW_LIFT, TOP_LIFT, CLAW, WEBCAM_1;

    val s: String = this.name.replace("_", "").lowercase().let { if (it == "webcam1") "Webcam 1" else it }
}
