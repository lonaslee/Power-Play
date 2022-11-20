package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.*

@TeleOp
class MotorTest : OpMode() {
    private lateinit var low: DcMotorEx
    private lateinit var top: DcMotorEx
    private lateinit var claw: Claw
    private lateinit var drive: SampleMecanumDrive
    private lateinit var gamepads: Pair<GamepadExt, GamepadExt>

    override fun init() {
        low = hardwareMap[Config.LOW_LIFT.s] as DcMotorEx
        top = hardwareMap[Config.TOP_LIFT.s] as DcMotorEx
        claw = Claw(hardwareMap, telemetry)
        drive = SampleMecanumDrive(hardwareMap)
        gamepads = GamepadExt(gamepad1) to GamepadExt(gamepad2)
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    }

    override fun loop() {
        drive fieldcentricAccordingTo gamepads
//        arm adjustAccordingTo gamepads
        claw changeAccordingTo gamepads

        telemetry.addData("pos", low.currentPosition)

        gamepads.onEach { it.update() }
        telemetry.update()
    }
}
