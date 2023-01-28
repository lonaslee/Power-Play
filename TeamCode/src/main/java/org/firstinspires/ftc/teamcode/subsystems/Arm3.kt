package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.profile.AccelerationConstraint
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.TICKS_IN_DEGREES
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

    companion object {
        @JvmField var kCos = 0.15
        @JvmField var kP = 0.015
        @JvmField var kI = 0.0
        @JvmField var kD = 0.0

        @JvmField var mA = 600.0
        @JvmField var dA = 400.0
        @JvmField var mV = 800.0

        @JvmField var PERCENT = 0.7
        @JvmField var MULTIPLIER = 600
    }

    private var curState = MotionState(Arm.GROUND.toDouble(), 0.0, 0.0)
    private var profile =
        MotionProfileGenerator.generateMotionProfile(curState, curState, { mV }, { mA })

    private val timer = ElapsedTime()

    private var goingDown = false
    private var stackHeight = Arm.STACK
    override var state = Arm.GROUND
        set(value) {
            if (state == value) return
            goingDown = state > value

            if (goingDown && value == Arm.STACK && stackHeight > Arm.GROUND + 40) stackHeight -= 20

            timer.reset()

            val (start, goal) = (curState to MotionState(
                (if (value != Arm.STACK) value else stackHeight).toDouble(), 0.0, 0.0
            )).let { if (goingDown) it.first.flipped() to it.second.flipped() else it }

            println("VALUE : $value; BOOL : ${value in listOf(Arm.GROUND, Arm.HIGH, Arm.BACKHIGH)}")
            profile = MotionProfileGenerator.generateMotionProfile(start,
                goal,
                { mV },
                if (value in listOf(
                        Arm.GROUND, Arm.HIGH, Arm.BACKHIGH
                    )
                ) object : AccelerationConstraint {
                    val positiveOffset = if (start.x < 0) -1 * start.x else 0.0
                    val totalDistance = ((goal.x + positiveOffset) - (start.x + positiveOffset))

                    override fun get(s: Double) =
                        ((s + positiveOffset) / totalDistance).let { percent ->
                            if (percent < PERCENT) mA else MULTIPLIER * ((1 - percent) / 0.3)
                        }.also { print(".") }
                } else AccelerationConstraint { (if (goingDown) dA else mA).also { print("#") } })
                .let { if (goingDown) it.flipped() else it }

            field = value
        }

    fun update() {
        control.setPID(kP, kI, kD)

        val curPos = low.currentPosition - 170.0
        curState = profile[timer.time()]

        val pow = control.calculate(
            curPos, curState.x
        ) + kCos * cos(Math.toRadians(state / TICKS_IN_DEGREES))

        motors.forEach { it.power = pow }

        telemetry?.addData("stack", stackHeight)

        telemetry?.addData("_currentPos", curPos)
        telemetry?.addData("_targetPos", curState.x)
        telemetry?.addData("angle", curPos / TICKS_IN_DEGREES)
        telemetry?.addData("pow", pow)
    }
}
