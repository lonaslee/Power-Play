package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.HIGH
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.GROUND
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.LOW
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.MID
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.DriveExt
import org.openftc.easyopencv.OpenCvWebcam

@TeleOp
class TeleOp1 : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var drive: DriveExt
    private lateinit var webcam: OpenCvWebcam

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun runOpMode() {
        arm = Arm(hardwareMap, tm)
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
            onPressed(gp1::a, gp2::a) { arm.state = GROUND }
            onPressed(gp1::x, gp2::x) { arm.state = LOW }
            onPressed(gp1::y, gp2::y) { arm.state = MID }
            onPressed(gp1::b, gp2::b) { arm.state = HIGH }
        }.also {
            waitForStart()
            it.run()
        }
    }
}
