package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.cos

@com.acmerobotics.dashboard.config.Config
open class Arm(
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
        @JvmField var STACK = -100
        const val LOW = 10
        const val MID = 100
        const val HIGH = 210
        @JvmField var BACKHIGH = 280
        const val BACKMID = 365
        const val BACKLOW = 470
        const val BACKGROUND = 600

        const val LOWER = Int.MAX_VALUE
        @JvmField var LOWINC = 20

        @JvmField var kP = 0.015
        @JvmField var kI = 0.0
        @JvmField var kD = 0.0

        @JvmField var dP = 0.002
        @JvmField var dI = 0.0
        @JvmField var dD = 0.0003
        @JvmField var dF = -0.0002

        @JvmField var kCos = 0.15
        @JvmField var dCos = 0.01

        override val all =
            listOf(GROUND, STACK, LOW, MID, HIGH, BACKHIGH, BACKMID, BACKLOW, BACKGROUND)

        override fun next(this_: Number) = super.next(this_).toInt()
        override fun prev(this_: Number) = super.prev(this_).toInt()

        const val TICKS_IN_DEGREES = 220 / 90.0
    }


    private var goingDown = false
    private var stackHeight = STACK
    override var state = GROUND
        set(value) {
            if (state == value) return

            goingDown = state > value
            if (goingDown && value == STACK) stackHeight -= 20

            if (value == LOWER) field += if (field < HIGH) -LOWINC else LOWINC
            else field = value
        }

    open fun update() {
        control.setPID(kP, kI, kD)
        downControl.setPIDF(dP, dI, dD, dF)

        val curPos = low.currentPosition - 170.0

        val pow = if (goingDown && (state == GROUND || state == STACK)) {
            downControl.calculate(
                curPos, if (state == GROUND) state.toDouble() else stackHeight.toDouble()
            ) + dCos * cos(Math.toRadians(state / TICKS_IN_DEGREES))
        } else {
            control.calculate(
                curPos, state.toDouble()
            ) + kCos * cos(Math.toRadians(state / TICKS_IN_DEGREES))
        }

        motors.forEach { it.power = pow }

        telemetry?.addData("stack", stackHeight)

        telemetry?.addData("_currentPos", curPos)
        telemetry?.addData("_targetPos", state)
        telemetry?.addData("angle", curPos / TICKS_IN_DEGREES)
        telemetry?.addData("pow", pow)
    }
}
