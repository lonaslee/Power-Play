package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robot.*
import org.firstinspires.ftc.teamcode.robot.Arm.States.BACKMID
import org.firstinspires.ftc.teamcode.robot.Arm.States.GROUND
import org.firstinspires.ftc.teamcode.robot.Arm.States.LOW
import org.firstinspires.ftc.teamcode.robot.Arm.States.MID
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
            updates += mutableListOf(arm::update, { drive.update(gamepads) }, {
                if (claw.state == Claw.States.OPENED && arm.state > MID) claw.halfOpen()
                else if (claw.state == Claw.States.HALF_OPENED && arm.state <= MID) claw.open()
            }, gamepads::sync)

            onAnyPressed(gp1::left_bumper, gp2::left_bumper) { claw.change() }
            onAnyPressed(gp1::dpad_up, gp2::dpad_up) { arm.state = Arm.States.next(arm.state).toInt() }
            onAnyPressed(gp1::dpad_down, gp2::dpad_down) { arm.state = Arm.States.prev(arm.state).toInt() }
            onAnyPressed(gp1::a, gp2::a) { arm.state = GROUND }
            onAnyPressed(gp1::x, gp2::x) { arm.state = LOW }
            onAnyPressed(gp1::y, gp2::y) { arm.state = MID }
            onAnyPressed(gp1::b, gp2::b) { arm.state = BACKMID }
        }.run()
    }
}
