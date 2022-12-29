package org.firstinspires.ftc.teamcode.robot

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

    object States : Subsystem.States {
        const val OPENED = 0.56
        const val CLOSED = 0.69
        const val HALF_OPENED = 0.6
        override val all = listOf(CLOSED, HALF_OPENED, OPENED)
    }

    override val state
        get() = when (servo.position) {
            in States.OPENED - 0.01..States.OPENED + 0.01 -> States.OPENED
            in States.CLOSED - 0.01..States.CLOSED + 0.01 -> States.CLOSED
            else                                          -> States.HALF_OPENED
        }

    fun open() {
        servo.position = States.OPENED
    }

    fun halfOpen() {
        servo.position = States.HALF_OPENED
    }

    fun close() {
        servo.position = States.CLOSED
    }

    fun change() {
        if (state == States.CLOSED) open()
        else close()
    }
}
