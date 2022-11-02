package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.drive.ArmConstants
import kotlin.math.cos


class Arm(hardwareMap: HardwareMap, private val telemetry: Telemetry) {
    val motor = hardwareMap[motorName] as DcMotorEx
    private val timer = ElapsedTime()

    init {
        motor.direction = DcMotorSimple.Direction.REVERSE
        motor.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
        motor.mode = RunMode.RUN_USING_ENCODER
        motor.mode = RunMode.STOP_AND_RESET_ENCODER
        motor.mode = RunMode.RUN_USING_ENCODER
    }

    fun up() {
        reference = Height.next(reference)
        telemetry.putfs("ARM UP - $reference")
    }

    fun down() {
        reference = Height.prev(reference)
        telemetry.putfs("ARM DOWN - $reference")
    }

    var reference = Height.FLR
        set(value) {
            timer.reset()
            integralSum = 0.0
            lastError = 0
            field = value
        }

    private var lastError = 0
    private var integralSum = 0.0
    private val TICKS_IN_DEG = 751.8 / 180

    fun update() {
        val error = reference.pos - motor.currentPosition
        val derivative = (error - lastError) / timer.seconds()
        integralSum += error * timer.seconds()
        val pid =
            (ArmConstants.kP * error) + (ArmConstants.kI * integralSum) + (ArmConstants.kD * derivative)
        val ff = cos(Math.toRadians(reference.pos / TICKS_IN_DEG)) * ArmConstants.kCos

        telemetry.addData("pid", pid)
        telemetry.addData("ff", ff)
        telemetry.addData("pidf", pid + ff)
        motor.power = pid + ff
        lastError = error
        timer.reset()
    }

    companion object {
        const val motorName = "lift"

        enum class Height(val pos: Int) {
            TOP(300), MID(220), LOW(130), FLR(0);

            companion object {
                fun next(cur: Height) = when (cur) {
                    LOW -> MID
                    FLR -> LOW
                    else -> TOP
                }

                fun prev(cur: Height) = when (cur) {
                    TOP -> MID
                    MID -> LOW
                    else -> FLR
                }
            }
        }
    }
}