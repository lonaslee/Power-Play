package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robot.EventLoop
import org.firstinspires.ftc.teamcode.robot.GamepadExt
import org.firstinspires.ftc.teamcode.robot.subsystems.Arm2
import org.firstinspires.ftc.teamcode.robot.subsystems.Claw
import org.firstinspires.ftc.teamcode.robot.subsystems.DriveExt
import org.firstinspires.ftc.teamcode.robot.sync

@TeleOp
class TeleOp2 : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm2
    private lateinit var drive: DriveExt
    private lateinit var gamepads: Pair<GamepadExt, GamepadExt>

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun runOpMode() {
        gamepads = GamepadExt(gamepad1) to GamepadExt(gamepad2)
        arm = Arm2(hardwareMap, tm)
        claw = Claw(hardwareMap)
        drive = DriveExt(hardwareMap)

        gamepads = GamepadExt(gamepad1) to GamepadExt(gamepad2)

        EventLoop(::opModeIsActive).apply {
            onPressed(gamepads.first::dpad_up) { arm.state = Arm2.next(arm.state) }
            onPressed(gamepads.first::dpad_down) { arm.state = Arm2.prev(arm.state) }

            onPressed(gamepads.first::left_bumper) { claw.change() }

            updates += listOf({ drive.update(gamepads) }, arm::update, { tm.update(); Unit }, gamepads::sync)
        }.also {
            waitForStart()
            it.run()
        }
    }
}
