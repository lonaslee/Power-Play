package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robot.EventLoop
import org.firstinspires.ftc.teamcode.robot.GamepadExt
import org.firstinspires.ftc.teamcode.robot.subsystems.Arm
import org.firstinspires.ftc.teamcode.robot.subsystems.Arm.States.BACKMID
import org.firstinspires.ftc.teamcode.robot.subsystems.Arm.States.GROUND
import org.firstinspires.ftc.teamcode.robot.subsystems.Arm.States.LOW
import org.firstinspires.ftc.teamcode.robot.subsystems.Arm.States.MID
import org.firstinspires.ftc.teamcode.robot.subsystems.Claw
import org.firstinspires.ftc.teamcode.robot.subsystems.DriveExt
import org.firstinspires.ftc.teamcode.robot.sync
import org.openftc.easyopencv.OpenCvWebcam

@TeleOp
class TeleOp1 : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var drive: DriveExt
    private lateinit var webcam: OpenCvWebcam

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun runOpMode() {
        arm = Arm(hardwareMap)
        claw = Claw(hardwareMap)
        drive = DriveExt(hardwareMap)

        val gp1 = GamepadExt(gamepad1)
        val gp2 = GamepadExt(gamepad2)
        val gamepads = gp1 to gp2

        EventLoop(::opModeIsActive).apply {
            updates += listOf(arm::update, { drive.update(gamepads) }, {
                if (claw.state == Claw.OPENED && arm.state > MID) claw.state = Claw.HALF_OPENED
                else if (claw.state == Claw.HALF_OPENED && arm.state <= MID) claw.state = Claw.OPENED
            }, gamepads::sync)

            onAnyPressed(gp1::left_bumper, gp2::left_bumper) { claw.change() }
            onAnyPressed(gp1::dpad_up, gp2::dpad_up) { arm.state = Arm.next(arm.state) }
            onAnyPressed(gp1::dpad_down, gp2::dpad_down) { arm.state = Arm.prev(arm.state) }
            onAnyPressed(gp1::a, gp2::a) { arm.state = GROUND }
            onAnyPressed(gp1::x, gp2::x) { arm.state = LOW }
            onAnyPressed(gp1::y, gp2::y) { arm.state = MID }
            onAnyPressed(gp1::b, gp2::b) { arm.state = BACKMID }
        }.also {
            waitForStart()
            it.run()
        }
    }
}
