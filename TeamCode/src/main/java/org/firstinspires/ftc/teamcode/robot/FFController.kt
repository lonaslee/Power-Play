package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs

class FFController(var coefs: Coefficients) {
    data class Coefficients(val kA: Double, val kV: Double)

    private val timer = ElapsedTime()
    private var timeDelta: Double
        get() = timer.seconds()
        set(value) = if (value == 0.0) timer.reset() else throw IllegalArgumentException()

    companion object {
        var MAX_VELO = 0.3
        var MAX_ACCEL = 0.3
    }

    private var lastoutput: Int = 0
    private var curoutput: Int = 0

    private val curvelo
        get() = (curoutput - lastoutput) / timeDelta

    private var lastvelo: Double = 0.0

    var setpoint = 0

    val error: Int
        get() = setpoint - curoutput

    fun calculate(output: Int): Double {
        curoutput = output

        val (desiredVelo, desiredAccel) = profile

        val veloErr = desiredVelo - curvelo

        val accel = (curvelo - lastvelo) / timeDelta
        val accelErr = desiredAccel - accel

        val input = coefs.kV * veloErr + coefs.kA * accelErr

        lastoutput = output
        lastvelo = curvelo
        timeDelta = 0.0

        return input
    }


    private val profile: Pair<Double, Double>
        get() {
            var desiredVelo: Double
            var desiredAccel: Double

            val posErr = setpoint - curoutput
            val direction = if (posErr < 0) -1 else 1

            if (MAX_VELO > abs(curvelo)) {
                desiredVelo = curvelo + direction * MAX_ACCEL * timeDelta
                desiredAccel = MAX_ACCEL
            } else {
                desiredVelo = MAX_VELO
                desiredAccel = 0.0
            }
            if (error <= (desiredVelo * desiredVelo) / (2 * MAX_ACCEL)) {
                desiredVelo = curvelo - direction * MAX_ACCEL * timeDelta
                desiredAccel = -MAX_ACCEL
            }
            return desiredVelo to desiredAccel
        }
}
