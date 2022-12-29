package org.firstinspires.ftc.teamcode.robot.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry

class Claw(
    hardwareMap: HardwareMap, private val telemetry: Telemetry? = null
) : Subsystem {
    private val servo = (hardwareMap.get(RobotConfig.CLAW.s) as ServoImplEx).apply {
        direction = Servo.Direction.FORWARD
    }

    companion object States : Subsystem.States {
        const val OPENED = 0.56
        const val CLOSED = 0.69
        const val HALF_OPENED = 0.6
        override val all = listOf(CLOSED, HALF_OPENED, OPENED)
    }

    override val state
        get() = when (servo.position) {
            in OPENED - 0.01..OPENED + 0.01 -> OPENED
            in CLOSED - 0.01..CLOSED + 0.01 -> CLOSED
            else                            -> HALF_OPENED
        }

    fun open() {
        servo.position = OPENED
    }

    fun halfOpen() {
        servo.position = HALF_OPENED
    }

    fun close() {
        servo.position = CLOSED
    }

    fun change() {
        if (state == CLOSED) open()
        else close()
    }
}
