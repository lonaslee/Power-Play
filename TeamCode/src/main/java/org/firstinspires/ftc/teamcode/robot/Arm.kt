package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class Arm(hardwareMap: HardwareMap, private val telemetry: Telemetry) {
    val topmotor = hardwareMap[topMotorName] as DcMotorEx
    val lowmotor = hardwareMap[lowMotorName] as DcMotorEx
    val motors =
        listOf(topmotor, lowmotor).onEach { it.direction = DcMotorSimple.Direction.REVERSE }

    var height = Height.FLR

    fun incUp() {
        telemetry.addLine("incUp : ${lowmotor.currentPosition}")
        motors.forEach {
            it.targetPosition = it.currentPosition + 30
            it.power = 1.0
            it.mode = DcMotor.RunMode.RUN_TO_POSITION
        }
    }

    fun incDown() {
        telemetry.addLine("incUp : ${lowmotor.currentPosition}")
        motors.forEach {
            it.targetPosition = it.currentPosition - 20
            it.power = 0.8
            it.mode = DcMotor.RunMode.RUN_TO_POSITION
        }
    }

    fun up() {
        height = Height.next(height)
        motors.forEach {
            it.targetPosition = height.pos
            it.power = 0.5
            it.mode = DcMotor.RunMode.RUN_TO_POSITION
        }
    }

    fun down() {
        height = Height.prev(height)
        motors.forEach {
            it.targetPosition = height.pos
            it.power = .5
            it.mode = DcMotor.RunMode.RUN_TO_POSITION
        }
    }

    companion object {
        const val lowMotorName = "lowlift"
        const val topMotorName = "toplift"

        enum class Height(val pos: Int) {
            TOP(240), MID(240), LOW(150), FLR(5);

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
