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

        val pid = control.calculate(low.currentPosition.toDouble(), setpoint.toDouble())
        val ff = kCos * cos(Math.toRadians((setpoint - 160) / TICKS_IN_DEGREES))

        motors.forEach { it.power = pid + ff }


        tm.addData("_currentPos", low.currentPosition)
        tm.addData("_targetPos", setpoint)
        tm.addData("angle", low.currentPosition / TICKS_IN_DEGREES)
        tm.addData("pidf", pid + ff)
        tm.update()
    }

    companion object {
        @JvmField var kP = 0.0
        @JvmField var kI = 0.0
        @JvmField var kD = 0.0
        @JvmField var kCos = 0.0
        @JvmField var setpoint = 0
        const val TICKS_IN_DEGREES = 260 / 90.0
    }
}
