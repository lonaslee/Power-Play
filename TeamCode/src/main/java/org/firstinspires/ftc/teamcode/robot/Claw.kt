package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry

class Claw(
    hardwareMap: HardwareMap, private val telemetry: Telemetry? = null, private val arm: Arm? = null
) {
    private val servo = (hardwareMap.get(RobotConfig.CLAW.s) as ServoImplEx).apply {
        direction = Servo.Direction.FORWARD
    }

    companion object {
        const val OPEN_POS = 0.56
        const val CLOSE_POS = 0.66
        const val HALF_OPEN_POS = 0.6
    }

    fun open() {
        telemetry?.addLine("ARM OPEN")
        val armBack = arm?.let { it.height > Arm.Height.MID } ?: false
        servo.position = if (armBack) HALF_OPEN_POS else OPEN_POS
    }

    fun close() {
        telemetry?.addLine("ARM CLOSE")
        servo.position = CLOSE_POS
    }

    fun change() {
        if (servo.position == CLOSE_POS) open()
        else close()
    }

    fun changeAccordingTo(gp1: GamepadExt, gp2: GamepadExt) {
        if (gp1 pressed gp1::left_bumper || gp2 pressed gp2::left_bumper) change()
    }

    infix fun changeAccordingTo(gps: Pair<GamepadExt, GamepadExt>) =
        changeAccordingTo(gps.first, gps.second)
}
