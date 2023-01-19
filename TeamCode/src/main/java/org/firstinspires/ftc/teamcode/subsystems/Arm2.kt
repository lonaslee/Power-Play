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

    companion object {
        @JvmField var mV = 600.0
        @JvmField var mA = 600.0
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
                MotionState((if (value != STACK) value else stackHeight).toDouble(), 0.0, 0.0),
                mV,
                mA
            )
            field = value
        }

    fun update() {
        control.setPID(kP, kI, kD)
        downControl.setPIDF(dP, dI, dD, dF)

        val curPos = low.currentPosition - 170.0
        val profileState = profile[timer.time()]

        val pow = (if (goingDown && (state == GROUND || state == STACK)) downControl to dCos
        else control to kCos).let { (controller, cosConstant) ->
            controller.calculate(
                curPos, profileState.x
            ) + cosConstant * cos(Math.toRadians(state / TICKS_IN_DEGREES))
        }

        motors.forEach { it.power = pow }

        telemetry?.addData("stack", stackHeight)

        telemetry?.addData("_currentPos", curPos)
        telemetry?.addData("_targetPos", profileState.x)
        telemetry?.addData("angle", curPos / TICKS_IN_DEGREES)
        telemetry?.addData("pow", pow)
    }
}
