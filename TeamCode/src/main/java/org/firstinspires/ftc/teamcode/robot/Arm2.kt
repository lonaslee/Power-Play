package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.drive.ArmConstants
import kotlin.math.cos


class Arm2(hardwareMap: HardwareMap, private val telemetry: Telemetry) {
    val lowMotor = hardwareMap[lowMotorName] as DcMotorEx
    val topMotor = hardwareMap[topMotorName] as DcMotorEx
    val motors = listOf(lowMotor, topMotor)
    private val timer = ElapsedTime()

    init {
        motors.forEach {
            it.direction = DcMotorSimple.Direction.REVERSE
            it.mode = RunMode.STOP_AND_RESET_ENCODER
            it.mode = RunMode.RUN_WITHOUT_ENCODER
        }
    }

    fun up() {
        reference = Height.next(reference)
    }

    fun down() {
        reference = Height.prev(reference)
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
    private val TICKS_IN_DEG = 537.6 / 180

    fun update() {
        val error = reference.pos - lowMotor.currentPosition
        val derivative = (error - lastError) / timer.seconds()
        integralSum += error * timer.seconds()
        val pid =
            (ArmConstants.kP * error) + (ArmConstants.kI * integralSum) + (ArmConstants.kD * derivative)
        val ff = cos(Math.toRadians(reference.pos / TICKS_IN_DEG)) * ArmConstants.kCos
        val pow = pid + ff

        telemetry.addData("pid", pid)
        telemetry.addData("ff", ff)
        telemetry.addData("pos", lowMotor.currentPosition)
        telemetry.addData("target", reference.pos)
        motors.forEach { it.power = pow }
        lastError = error
        timer.reset()
    }

    companion object {
        const val lowMotorName = "lowlift"
        const val topMotorName = "toplift"

        enum class Height(val pos: Int) {
            TOP(240), MID(240), LOW(150), FLR(0);

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