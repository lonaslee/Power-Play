package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry

class Claw(hardwareMap: HardwareMap, private val telemetry: Telemetry) {
    private val motor = hardwareMap.get(motorName) as ServoImplEx

    init {
        motor.direction = Servo.Direction.FORWARD
    }

    var opened: Boolean = true

    fun open() {
        if (!opened) {
            motor.position = 0.52
            opened = true
        }
    }

    fun close() {
        if (opened) {
            motor.position = 0.64
            opened = false
        }
    }

    fun change() {
        if (opened) close()
        else open()
    }

    companion object {
        const val motorName = "claw"
    }
}