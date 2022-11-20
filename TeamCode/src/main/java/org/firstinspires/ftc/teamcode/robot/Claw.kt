package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry

class Claw(hardwareMap: HardwareMap, private val telemetry: Telemetry) {
    private val servo = hardwareMap.get(Config.CLAW.s) as ServoImplEx

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

    fun changeAccordingTo(gp1: GamepadExt, gp2: GamepadExt) {
        if (gp1 pressed gp1::b || gp2 pressed gp2::b) change()
    }

    infix fun changeAccordingTo(gps: Pair<GamepadExt, GamepadExt>) = changeAccordingTo(gps.first, gps.second)
}
