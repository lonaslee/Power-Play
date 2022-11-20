package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.util.ElapsedTime

class PIDController(var coefs: Coefficients) {
    data class Coefficients(val kP: Double, val kI: Double, val kD: Double, val kCos: Double)
    private val timer = ElapsedTime()

    var setpoint = 0
        set(value) {
            integralsum = 0.0
            lasterror = 0
            field = value
        }

    private var error = 0
    private var lasterror = error

    private var integralsum = 0.0

    fun calculate(output: Int): Pair<Double, Triple<Double, Double, Double>> {
        error = setpoint - output
        integralsum += error * timer.seconds()

        val pInput = coefs.kP * error
        val dInput = coefs.kD * ((error - lasterror) / timer.seconds())
        val iInput = coefs.kI * integralsum

        lasterror = error
        timer.reset()

        return Triple(pInput, iInput, dInput).let {
            (it.first + it.second + it.third) to it
        }
    }
}
