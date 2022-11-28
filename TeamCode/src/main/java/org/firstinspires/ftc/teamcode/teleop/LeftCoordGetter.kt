package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.autonomous.Trajectories
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.*

@TeleOp
class LeftCoordGetter : OpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var drive: SampleMecanumDrive
    private lateinit var gamepads: Pair<GamepadExt, GamepadExt>

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun init() {
        arm = Arm(hardwareMap)
        claw = Claw(hardwareMap, arm = arm)
        drive = SampleMecanumDrive(hardwareMap).apply { poseEstimate = Trajectories.leftStartPos }
        gamepads = GamepadExt(gamepad1) to GamepadExt(gamepad2)
    }

    override fun loop() {
        drive fieldcentricAccordingTo gamepads
        arm adjustAccordingTo gamepads
        claw changeAccordingTo gamepads

        val (x, y, heading) = drive.poseEstimate
        telemetry.addData("x", x)
        telemetry.addData("y", y)
        telemetry.addData("heading", heading)

        gamepads.onEach { it.update() }
        tm.update()
    }
}
