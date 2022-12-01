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

class Arm(hardwareMap: HardwareMap, private val telemetry: Telemetry? = null) {
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
    var height = GROUND
        set(value) {
            if (height == value) return
            goingDown = (height > value) || (value == STACK && height != GROUND)
            field = value
        }

    fun update() {
        val curPos = low.currentPosition - 170.0

        val pow = if (goingDown && (height == GROUND || height == STACK)) {
            downControl.calculate(
                curPos, height.pos.toDouble()
            ) + dCos * cos(Math.toRadians(height.pos / TICKS_IN_DEGREES)).also { println("DOWN WITH ${height.name}") }
        } else {
            control.calculate(
                curPos, height.pos.toDouble()
            ) + kCos * cos(Math.toRadians(height.pos / TICKS_IN_DEGREES))
        }

        motors.forEach { it.power = pow }

        telemetry?.addData("_currentPos", curPos)
        telemetry?.addData("_targetPos", height.pos)
        telemetry?.addData("angle", curPos / TICKS_IN_DEGREES)
        telemetry?.addData("pow", pow)
    }

    fun adjustAccordingTo(gp1: GamepadExt, gp2: GamepadExt) {
        if (gp1 pressed gp1::dpad_up || gp2 pressed gp2::dpad_up) height = height.next
        else if (gp1 pressed gp1::dpad_down || gp2 pressed gp2::dpad_down) height = height.prev
        else if (gp1 pressed gp1::a || gp2 pressed gp2::a) height = GROUND
        else if (gp1 pressed gp1::x || gp2 pressed gp2::x) height = LOW
        else if (gp1 pressed gp1::y || gp2 pressed gp2::y) height = MID
        else if (gp1 pressed gp1::b || gp2 pressed gp2::b) height = BACKMID
        update()
    }

    infix fun adjustAccordingTo(gps: Pair<GamepadExt, GamepadExt>) =
        adjustAccordingTo(gps.component1(), gps.component2())

    enum class Height(val pos: Int) {
        GROUND(-170), STACK(-65), LOW(70), MID(160), BACKMID(320), BACKLOW(400);

        val next get() = values()[if (ordinal > 4) 4 else ordinal + 1]
        val prev get() = values()[if (ordinal == 0) 0 else ordinal - 1]
    }
}
