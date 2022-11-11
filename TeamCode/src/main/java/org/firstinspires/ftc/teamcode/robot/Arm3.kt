package org.firstinspires.ftc.teamcode.robot

import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class Arm3(hardwareMap: HardwareMap, private val telemetry: Telemetry) {
    private val topmotor = hardwareMap[topMotorName] as DcMotorEx
    private val lowmotor = hardwareMap[lowMotorName] as DcMotorEx
    private val motors =
        listOf(topmotor, lowmotor).onEach {
            it.direction = DcMotorSimple.Direction.REVERSE
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }

    private val control = PIDFController(kP, kI, kD, kF)

    var height = Height.FLR

    fun up() {
        height = Height.next(height)
    }

    fun down() {
        height = Height.prev(height)
    }

    fun update() {
        val armPos = lowmotor.currentPosition
        val pow = control.calculate(armPos.toDouble(), height.pos.toDouble())
        motors.forEach { it.power = pow }

        telemetry.addData("pos", armPos)
        telemetry.addData("ref", height.pos)
        telemetry.addData("pow", pow)
    }

    companion object {
        const val lowMotorName = "lowlift"
        const val topMotorName = "toplift"
        const val kP = 0.001
        const val kI = 0.0
        const val kD = 0.0
        const val kF = 0.000007

        enum class Height(val pos: Int) {
            TOP(240), MID(240), LOW(160), FLR(5);

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
