package org.firstinspires.ftc.teamcode.robot

import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.Arm.Height.*
import kotlin.math.cos

class Arm(
    hardwareMap: HardwareMap,
    private val gamepads: Pair<GamepadExt, GamepadExt>? = null,
    private val telemetry: Telemetry? = null
) : Subsystem {
    constructor(hardwareMap: HardwareMap, telemetry: Telemetry?) : this(
        hardwareMap, null, telemetry
    )

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

    companion object {
        const val kCos = 0.13
        const val dCos = 0.01
        const val TICKS_IN_DEGREES = 220 / 90.0
    }

    private var goingDown = false
    private var stackHeight = STACK.pos
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

    private fun updatePID() {
        val curPos = low.currentPosition - 170.0

        val pow = if (goingDown && (state == GROUND || state == STACK)) {
            downControl.calculate(
                curPos, if (state == GROUND) state.pos.toDouble() else stackHeight.toDouble()
            ) + dCos * cos(Math.toRadians(state.pos / TICKS_IN_DEGREES)).also {
                println("DOWN WITH ${state.name}")
            }
        } else {
            control.calculate(
                curPos, state.pos.toDouble()
            ) + kCos * cos(Math.toRadians(state.pos / TICKS_IN_DEGREES))
        }

        motors.forEach { it.power = pow }

        telemetry?.addData("stack", stackHeight)

        telemetry?.addData("_currentPos", curPos)
        telemetry?.addData("_targetPos", state.pos)
        telemetry?.addData("angle", curPos / TICKS_IN_DEGREES)
        telemetry?.addData("pow", pow)
    }

    fun update(gp1: GamepadExt, gp2: GamepadExt) {
        if (anypressed(gp1::dpad_up, gp2::dpad_up)) state = state.next
        else if (anypressed(gp1::dpad_down, gp2::dpad_down)) state = state.prev
        else if (anypressed(gp1::a, gp2::a)) state = GROUND
        else if (anypressed(gp1::x, gp2::x)) state = LOW
        else if (anypressed(gp1::y, gp2::y)) state = MID
        else if (anypressed(gp1::b, gp2::b)) state = BACKMID
        updatePID()
    }

    override fun update() =
        if (gamepads == null) updatePID() else update(gamepads.first, gamepads.second)

    enum class Height(var pos: Int) {
        GROUND(-170), STACK(-70), LOW(70), MID(160), BACKMID(360), BACKLOW(400);

        val next get() = values()[if (ordinal > 4) 4 else ordinal + 1]
        val prev get() = values()[if (ordinal == 0) 0 else ordinal - 1]
    }
}
