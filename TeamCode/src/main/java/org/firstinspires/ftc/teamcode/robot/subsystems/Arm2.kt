package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.cos

@com.acmerobotics.dashboard.config.Config
class Arm2(
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
        const val MID = 160
        const val BACKMID = 360
        const val BACKLOW = 400

        override val all = listOf(GROUND, STACK, LOW, MID, BACKMID, BACKLOW)
        override fun next(this_: Number) = super.next(this_).toInt()
        override fun prev(this_: Number) = super.prev(this_).toInt()

        @JvmField var mV = 15.0
        @JvmField var mA = 15.0
    }

    private var profile = MotionProfileGenerator.generateSimpleMotionProfile(
        MotionState(GROUND.toDouble(), 0.0, 0.0), MotionState(GROUND.toDouble(), 0.0, 0.0), mV, mA
    )
    private val timer = ElapsedTime()

    private var goingDown = false
    private var stackHeight = STACK
    override var state = GROUND
        set(value) {
            if (state == value) return
            goingDown = state > value
            if (goingDown && value == STACK && STACK > GROUND + 40) {
                stackHeight -= 20
                println("SUBTRACT -> $stackHeight")
            }
            timer.reset()
            profile = MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(state.toDouble(), 0.0, 0.0),
                MotionState((if (value != STACK) value else stackHeight).toDouble(), 0.0, 0.0), mV, mA
            )
            field = value
        }

    private object PIDConstants {
        const val kCos = 0.13
        const val dCos = 0.01
        const val TICKS_IN_DEGREES = 220 / 90.0
    }

    fun update() {
        val curPos = low.currentPosition - 170.0
        val profileState = profile[timer.time()]

        val pow = (if (goingDown && (state == GROUND || state == STACK)) downControl to PIDConstants.dCos
        else control to PIDConstants.kCos).let { (controller, cosConstant) ->
            controller.calculate(
                curPos, profileState.x
            ) + cosConstant * cos(
                Math.toRadians(state / PIDConstants.TICKS_IN_DEGREES)
            )
        }

        motors.forEach { it.power = pow }

        telemetry?.addData("stack", stackHeight)

        telemetry?.addData("_currentPos", curPos)
        telemetry?.addData("_targetPos", state)
        telemetry?.addData("angle", curPos / PIDConstants.TICKS_IN_DEGREES)
        telemetry?.addData("pow", pow)
    }
}
