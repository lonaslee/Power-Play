package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.profile.MotionProfileGenerator.generateMotionProfile
import com.acmerobotics.roadrunner.profile.MotionState
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.PIDController
import java.lang.Math.toRadians
import java.util.LinkedList
import kotlin.math.abs
import kotlin.math.cos

@com.acmerobotics.dashboard.config.Config
open class Arm4(hardwareMap: HardwareMap, private val telemetry: Telemetry? = null) : Subsystem {
    private val topMotor = hardwareMap[RobotConfig.TOP_LIFT.s] as DcMotorEx
    private val lowMotor = hardwareMap[RobotConfig.LOW_LIFT.s] as DcMotorEx
    private val motors = listOf(topMotor, lowMotor).onEach {
        it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        it.direction = DcMotorSimple.Direction.REVERSE
        it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    private val voltageSensor = hardwareMap.voltageSensor.iterator().next()!!
    private var voltage = voltageSensor.voltage

    private val stack = LinkedList<Int>().apply {
        addAll(
            listOf(
                AnglePresets.i5,
                AnglePresets.j4,
                AnglePresets.k3,
                AnglePresets.l2,
                AnglePresets.GROUND
            )
        )
    }

    override var state: Int = AnglePresets.GROUND
        set(value) {
            if (field == value) return
            // if (inMotion() return

            timer.reset()
            voltage = voltageSensor.voltage

            val setpoint = if (value == AnglePresets.STACK) {
                stack.pollFirst() ?: value
            } else value

            val startPos = motionState.x
            val distance = abs(startPos - setpoint)

            motionProfile = generateMotionProfile(motionState,
                MotionState(setpoint.toDouble(), 0.0, 0.0),
                { eV },
                { s -> (1 - (abs(startPos - s) / distance)).let { fA * if (it > 0.2) 1.0 else it / 0.2 } })

            field = value
        }

    fun inMotion() = timer.time() < motionProfile.duration()

    private val controller = PIDController(aP, bI, cD)
    private val timer = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)
    private var motionState = MotionState(AnglePresets.GROUND.toDouble(), 0.0, 0.0)
    private var motionProfile = generateMotionProfile(motionState, motionState, { eV }, { fA })

    fun update() {
        controller.constants = Triple(aP, bI, cD)
        val encoderPosition = topMotor.currentPosition
        val angle = encoderPosition / gTICKS_IN_DEGREES + hANGDISP

        motionState = motionProfile[timer.time()]

        val pid = controller.calculate(motionState.x, angle)
        val ff = dCos * cos(toRadians(angle))

        val power = (pid + ff) * 12 / voltage
        motors.forEach { it.power = power }

        telemetry?.addData("encoderPos", encoderPosition)
        telemetry?.addData("_angle", angle)
        telemetry?.addData("_target", motionState.x)
        telemetry?.addData("state", state)
        telemetry?.addData("power", power)
    }

    companion object {
        @JvmField var aP = 0.0001
        @JvmField var bI = 0.0
        @JvmField var cD = 0.0
        @JvmField var dCos = 0.01

        @JvmField var eV = 600.0
        @JvmField var fA = 600.0

        /** angle / ticks */
        @JvmField var gTICKS_IN_DEGREES = 90 / 220.0

        /** difference from ground to 0, in degrees */
        @JvmField var hANGDISP = 70.0

    }
}
