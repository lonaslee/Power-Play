package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Arm2
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.DriveExt

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
        val gp1 = gamepads.first
        val gp2 = gamepads.second

        EventLoop(::opModeIsActive).apply {
            onPressed(gamepads.first::dpad_up) { arm.state = Arm.next(arm.state) }
            onPressed(gamepads.first::dpad_down) { arm.state = Arm.prev(arm.state) }

            onPressed(gp1::a, gp2::a) { arm.state = Arm.GROUND }
            onPressed(gp1::x, gp2::x) { arm.state = Arm.LOW }
            onPressed(gp1::y, gp2::y) { arm.state = Arm.MID }
            onPressed(gp1::b, gp2::b) { arm.state = Arm.HIGH }

            onPressed(gamepads.first::left_bumper) { claw.change() }

            updates += listOf(
                { drive.update(gamepads) },
                arm::update,
                { tm.update(); Unit },
                gamepads::sync
            )
        }.also {
            waitForStart()
            it.run()
        }
    }
}
