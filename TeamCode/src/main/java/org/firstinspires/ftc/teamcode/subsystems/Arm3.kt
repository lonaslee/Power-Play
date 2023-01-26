package org.firstinspires.ftc.teamcode.subsystems

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
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.GROUND
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.STACK
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.TICKS_IN_DEGREES
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.dCos
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.dD
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.dF
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.dI
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.dP
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.kCos
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.kD
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.kI
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.kP
import kotlin.math.cos

@com.acmerobotics.dashboard.config.Config
class Arm3(
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

    private val control = PIDController(kP, kI, kD)
    private val downControl = PIDFController(dP, dI, dD, dF)

    companion object {
        @JvmField var mA = 600.0
        @JvmField var mV = 800.0

        @JvmField var dA = 1400.0
        @JvmField var dV = 1400.0

        @JvmField var PERCENT = 0.8
        @JvmField var MULTIPLIER = 0.5
    }

    private var curState = MotionState(GROUND.toDouble(), 0.0, 0.0)
    private var profile =
        MotionProfileGenerator.generateSimpleMotionProfile(curState, curState, mV, mA)

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

            val goal = MotionState(
                (if (value != STACK) value else stackHeight).toDouble(), 0.0, 0.0
            )

            profile = if (goingDown) MotionProfileGenerator.generateSimpleMotionProfile(
                curState, goal, dV, dA
            )
            else MotionProfileGenerator.generateMotionProfile(curState, goal, { mV }, { s ->
                (mA * if (s / (goal.x - curState.x) > PERCENT) MULTIPLIER else 1.0)
                    .also { println("$s = GET ACCEL -> ${s / (goal.x - curState.x)} : $it") }
            })
            field = value
        }

    fun update() {
        control.setPID(kP, kI, kD)
        downControl.setPIDF(dP, dI, dD, dF)

        val curPos = low.currentPosition - 170.0
        curState = profile[timer.time()]

        val pow = (if (goingDown && (state == GROUND || state == STACK)) downControl to dCos
        else control to kCos).let { (controller, cosConstant) ->
            controller.calculate(
                curPos, curState.x
            ) + cosConstant * cos(Math.toRadians(state / TICKS_IN_DEGREES))
        }

        motors.forEach { it.power = pow }

        telemetry?.addData("stack", stackHeight)

        telemetry?.addData("_currentPos", curPos)
        telemetry?.addData("_targetPos", curState.x)
        telemetry?.addData("angle", curPos / TICKS_IN_DEGREES)
        telemetry?.addData("pow", pow)
    }
}
