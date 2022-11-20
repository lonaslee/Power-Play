package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.util.ElapsedTime

class FFController(var coefs: Coefficients) {
    data class Coefficients(val kA: Double, val kV: Double)

    private val timer = ElapsedTime()

    companion object {
        const val MAX_SPEED = 1
        const val MAX_ACCEL = 1
    }

    private var lastoutput: Int = 0
    private var curoutput: Int = 0

    private var curvelo = 0.0
    private var lastvelo: Double = 0.0

    var setpoint = 0

    fun calculate(output: Int): Double {
        curoutput = output

        curvelo = (curoutput - lastoutput) / timer.seconds()
        val veloErr = desiredVelo - curvelo

        val accel = (curvelo - lastvelo) / timer.seconds()
        val accelErr = desiredAccel - accel

        val input = coefs.kV * veloErr + coefs.kA * accelErr

        lastoutput = output
        lastvelo = curvelo
        timer.reset()

        return input
    }

    private val desiredVelo: Double
        get() {
            val posErr = setpoint - curoutput
            val direction = if (posErr < 0) -1 else 1
            TODO()
        }

    private val desiredAccel: Double
        get() {
            TODO()
        }
}
