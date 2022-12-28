package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.*

@TeleOp
class TeleOp2 : OpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var drive: SampleMecanumDrive
    private lateinit var gamepads: Pair<GamepadExt, GamepadExt>

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun init() {
        gamepads = GamepadExt(gamepad1) to GamepadExt(gamepad2)
        arm = Arm(hardwareMap, gamepads)
        claw = Claw(hardwareMap, gamepads, arm = arm)
        drive = SampleMecanumDrive(hardwareMap)
    }

    override fun loop() {
        drive.update(gamepads)
        arm.update()
        claw.update()

        gamepads.onEach { it.update() }
        tm.update()
    }
}
