package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.robot.RobotConfig
import kotlin.math.abs
import kotlin.math.cos

@TeleOp
@Config
class MotorTest : OpMode() {
    private lateinit var low: DcMotorEx
    private lateinit var top: DcMotorEx
    private lateinit var motors: List<DcMotorEx>

    private val control = PIDController(kP, kI, kD)
    private val tm = MultipleTelemetry(
        telemetry, FtcDashboard.getInstance().telemetry
    )

    override fun init() {
        low = hardwareMap[RobotConfig.LOW_LIFT.s] as DcMotorEx
        top = hardwareMap[RobotConfig.TOP_LIFT.s] as DcMotorEx
        motors = listOf(low, top).onEach {
            it.direction = DcMotorSimple.Direction.REVERSE
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    override fun loop() {
        control.setPID(kP, kI, kD)
        if (setpoint !in -170..390) return

        val curPos = (low.currentPosition - 170).toDouble()

        val pid = control.calculate(curPos, setpoint.toDouble())
        val ff = kCos * cos(Math.toRadians(setpoint / TICKS_IN_DEGREES))

        motors.forEach { it.power = pid + ff }

        tm.addData("_currentPos", curPos)
        tm.addData("_targetPos", setpoint)
        tm.addData("angle", curPos / TICKS_IN_DEGREES)
        tm.addData("pidf", pid + ff)
        tm.update()
    }

    companion object {
        @JvmField var kP = 0.009
        @JvmField var kI = 0.0
        @JvmField var kD = 0.00075
        @JvmField var kCos = 0.11
        @JvmField var setpoint = -170
        const val TICKS_IN_DEGREES = 220 / 90.0
    }
}
