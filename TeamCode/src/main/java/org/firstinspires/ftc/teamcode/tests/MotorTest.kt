package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.*

@TeleOp
class MotorTest : OpMode() {
    private lateinit var low: DcMotorEx
    private lateinit var top: DcMotorEx
    private lateinit var armMotors: List<DcMotorEx>
    private lateinit var claw: Claw
    private lateinit var drive: SampleMecanumDrive
    private lateinit var gamepads: Pair<GamepadExt, GamepadExt>

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun init() {
        low = hardwareMap[Config.LOW_LIFT.s] as DcMotorEx
        top = hardwareMap[Config.TOP_LIFT.s] as DcMotorEx
        armMotors = listOf(low, top).onEach {
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.direction = DcMotorSimple.Direction.REVERSE
        }
        claw = Claw(hardwareMap, tm)
        drive = SampleMecanumDrive(hardwareMap)
        gamepads = GamepadExt(gamepad1) to GamepadExt(gamepad2)
    }

    override fun loop() {
        drive fieldcentricAccordingTo gamepads
//        arm adjustAccordingTo gamepads
        claw changeAccordingTo gamepads

        tm.addData("pos", low.currentPosition)

        gamepads.onEach { it.update() }
        tm.update()
    }
}
