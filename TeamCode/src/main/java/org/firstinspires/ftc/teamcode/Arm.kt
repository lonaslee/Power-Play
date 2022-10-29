package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.cos

class Arm(hardwareMap: HardwareMap, private val telemetry: Telemetry) {
    private val motor = hardwareMap[motorName] as DcMotor

    private val timer = ElapsedTime()

    init {
        motor.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
        motor.mode = RunMode.RUN_WITHOUT_ENCODER
    }

    var reference: Double = 0.0
        set(value) {
            timer.reset()
            integralSum = 0.0
            lastError = 0.0
            field = value
        }

    var lastError = 0.0
    var integralSum = 0.0

    fun update() {
        val error = reference - motor.currentPosition
        val derivative = (error - lastError) / timer.seconds()
        integralSum += error * timer.seconds()
        val pid = (kP * error) + (kI * integralSum) + (kD * derivative)
        val ff = cos(reference) * kCos

        motor.power = pid + ff
        lastError = error
        timer.reset()
        telemetry.putfs("pow: ${pid + ff}\npid: $pid\nff: $ff")
    }

    companion object {
        const val motorName = "lift"

        const val kP = 0.0
        const val kI = 0.0
        const val kD = 0.0

        const val kCos = 0.0001
    }
}