package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx

class Claw(hardwareMap: HardwareMap) {
    private val servo = hardwareMap.get(motorName) as ServoImplEx

    init {
        servo.direction = Servo.Direction.FORWARD
    }

    var opened: Boolean = true

    fun open() {
        if (!opened) {
            servo.position = 0.52
            opened = true
        }
    }

    fun close() {
        if (opened) {
            servo.position = 0.66
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