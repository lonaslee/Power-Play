package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.*

@TeleOp(group = "test")
class TeleOp2 : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm3
    private lateinit var drive: DriveExt
    private lateinit var gamepads: Pair<GamepadExt, GamepadExt>

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun runOpMode() {
        gamepads = GamepadExt(gamepad1) to GamepadExt(gamepad2)
        arm = Arm3(hardwareMap, tm)
        claw = Claw(hardwareMap)
        drive = DriveExt(hardwareMap)

        val gp1 = gamepads.first
        val gp2 = gamepads.second

        EventLoop(::opModeIsActive, tm).apply {
            onPressed(gp1::dpad_up) { arm.state = Arm.next(arm.state) }
            onPressed(gp1::dpad_down) { arm.state = Arm.prev(arm.state) }

            onPressed(gp1::right_bumper) { arm.state = Arm.STACK }
            onPressed(gp1::a, gp2::a) { arm.state = Arm.GROUND }
            onPressed(gp1::x, gp2::x) { arm.state = Arm.MID }
            onPressed(gp1::y, gp2::y) { arm.state = Arm.HIGH }
            onPressed(gp1::b, gp2::b) { arm.state = Arm.BACKHIGH }

            onPressed(gp1::left_bumper, gp2::left_bumper) { claw.change() }

            updates += listOf({ drive.update(gamepads) },
                { arm.update() },
                { tm.update(); Unit },
                { gamepads.sync() })
        }.also {
            waitForStart()
            it.run()
        }
    }
}
