package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.cos

class Arm(
    hardwareMap: HardwareMap, private val telemetry: Telemetry? = null
) : Subsystem {
    private val low = hardwareMap[RobotConfig.LOW_LIFT.s] as DcMotorEx
    private val top = hardwareMap[RobotConfig.TOP_LIFT.s] as DcMotorEx
    private val motors = listOf(low, top).onEach {
        it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        it.direction = DcMotorSimple.Direction.REVERSE
    }

    private val control = PIDController(0.0085, 0.0, 0.0007)
    private val downControl = PIDFController(0.002, 0.0, 0.0006, 0.0)

    companion object States : Subsystem.States {
        const val GROUND = -170
        const val STACK = -70
        const val LOW = 70
        const val MID = 180
        const val BACKMID = 305
        const val BACKLOW = 400

        override val all = listOf(GROUND, STACK, LOW, MID, BACKMID, BACKLOW)
        override fun next(this_: Number) = super.next(this_).toInt()
        override fun prev(this_: Number) = super.prev(this_).toInt()
    }


    private var goingDown = false
    private var stackHeight = STACK
    override var state = GROUND
        set(value) {
            if (state == value) return
            goingDown = state > value
            if (goingDown && value == STACK) {
                stackHeight -= 20
                println("SUBTRACT")
            }
            field = value
        }

    private object PIDConstants {
        const val kCos = 0.13
        const val dCos = 0.01
        const val TICKS_IN_DEGREES = 220 / 90.0
    }

    fun update() {
        val curPos = low.currentPosition - 170.0

        val pow = if (goingDown && (state == GROUND || state == STACK)) {
            downControl.calculate(
                curPos, if (state == GROUND) state.toDouble() else stackHeight.toDouble()
            ) + PIDConstants.dCos * cos(Math.toRadians(state / PIDConstants.TICKS_IN_DEGREES))
        } else {
            control.calculate(
                curPos, state.toDouble()
            ) + PIDConstants.kCos * cos(Math.toRadians(state / PIDConstants.TICKS_IN_DEGREES))
        }

        motors.forEach { it.power = pow }

        telemetry?.addData("stack", stackHeight)

        telemetry?.addData("_currentPos", curPos)
        telemetry?.addData("_targetPos", state)
        telemetry?.addData("angle", curPos / PIDConstants.TICKS_IN_DEGREES)
        telemetry?.addData("pow", pow)
    }
}
