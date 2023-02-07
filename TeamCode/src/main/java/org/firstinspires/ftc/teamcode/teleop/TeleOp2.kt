package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.subsystems.*
import kotlin.system.measureNanoTime

@TeleOp(group = "test")
class TeleOp2 : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm4
    private lateinit var drive: DriveExt

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun runOpMode() {
        arm = Arm4(hardwareMap, tm)
        claw = Claw(hardwareMap)
        drive = DriveExt(hardwareMap)

        val gp1 = GamepadExt(gamepad1)
        val gp2 = GamepadExt(gamepad2)
        val gamepads = gp1 to gp2

        EventLoop(::opModeIsActive).apply {
            updates += listOf({ arm.update() },
                { drive.update(gamepads) },
                { gamepads.sync() },
                { tm.update() })

            onPressed(gp1::left_bumper, gp2::left_bumper) { claw.change() }

            onPressed(gp1::dpad_up, gp2::dpad_up) { arm.state = Arm.next(arm.state) }
            onPressed(gp1::dpad_down, gp2::dpad_down) { arm.state = Arm.prev(arm.state) }
            onPressed(gp1::a, gp2::a) { arm.state = Arm.GROUND }
            onPressed(gp1::x, gp2::x) { arm.state = Arm.MID }
            onPressed(gp1::y, gp2::y) { arm.state = Arm.HIGH }
            onPressed(gp1::b, gp2::b) { arm.state = Arm.BACKHIGH }
        }.also {
            waitForStart()
            it.run()
        }
    }
}
