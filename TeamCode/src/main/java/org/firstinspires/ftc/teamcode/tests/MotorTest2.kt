package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.robot.RobotConfig

@TeleOp
@Config
class MotorTest2 : OpMode() {
    private lateinit var low: DcMotorEx
    private lateinit var top: DcMotorEx
    private lateinit var motors: List<DcMotorEx>

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
        tm.addData("encoder", low.currentPosition)
        tm.addData("angle", low.currentPosition / TICKS_IN_DEGREES)
        tm.update()
    }

    companion object {
        const val TICKS_IN_DEGREES = 260 / 90.0
    }
}
