package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry

class OdoRetract(hardwareMap: HardwareMap, private val telemetry: Telemetry? = null) : Subsystem {
    private val servo = (hardwareMap[RobotConfig.RETRACTOR.s] as ServoImplEx).apply {
        direction = Servo.Direction.FORWARD
        position = DOWN
    }

    override var state = DOWN
        set(value) {
            if (field == value) return
            field = value
            servo.position = value
        }

    companion object : Subsystem.States {
        const val RETRACTED = 0.72
        const val DOWN = 0.45

        override val all = listOf(RETRACTED, DOWN)
    }
}
