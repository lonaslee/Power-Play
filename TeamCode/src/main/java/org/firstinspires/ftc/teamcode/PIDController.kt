package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.util.ElapsedTime

typealias PIDConstants = Triple<Double, Double, Double>

val PIDConstants.kP get() = first
val PIDConstants.kI get() = second
val PIDConstants.kD get() = third

class PIDController(var kP: Double, var kI: Double, var kD: Double) {
    constructor(constants: PIDConstants) : this(constants.kP, constants.kI, constants.kD)

    var constants = PIDConstants(kP, kI, kD)
        get() = PIDConstants(kP, kI, kD)
        set(value) {
            kP = value.kP
            kI = value.kI
            kD = value.kD
            field = value
        }

    var setpoint: Double = 0.0
        set(value) {
            if (field == value) return
            timer.reset()
            lastError = 0.0
            integralSum = 0.0
            field = value
        }

    private val timer = ElapsedTime()
    private var lastError = 0.0
    private var integralSum = 0.0

    fun calculate(setpoint: Double, output: Double): Double {
        this.setpoint = setpoint
        return calculate(output)
    }

    fun calculate(output: Double): Double {
        val error = setpoint - output
        val uP = kP * error

        val uD = kD * (error - lastError) / timer.milliseconds()

        integralSum += error * timer.milliseconds()
        val uI = kI * integralSum

        lastError = error
        timer.reset()
        return uP + uI + uD
    }
}
