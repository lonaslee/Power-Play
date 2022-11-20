package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.*

@TeleOp
class TeleOp4 : OpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm3
    private lateinit var drive: SampleMecanumDrive
    private lateinit var gamepads: Pair<GamepadExt, GamepadExt>

    override fun init() {
        claw = Claw(hardwareMap, telemetry)
        arm = Arm3(hardwareMap, telemetry)
        drive = SampleMecanumDrive(hardwareMap)
        gamepads = GamepadExt(gamepad1) to GamepadExt(gamepad2)
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    }

    override fun loop() {
        drive fieldcentricAccordingTo gamepads
        arm adjustAccordingTo gamepads
        claw changeAccordingTo gamepads

        gamepads.onEach { it.update() }
        telemetry.update()
    }
}
