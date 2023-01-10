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
        const val OPENED = 0.0
        const val CLOSED = 0.13
        override val all = listOf(CLOSED, OPENED)
    }

    override var state = OPENED
        set(value) {
            if (field != value) {
                field = value
                servo.position = value
            }
        }

    fun change() {
        state = if (state == CLOSED) OPENED else CLOSED
    }
}
