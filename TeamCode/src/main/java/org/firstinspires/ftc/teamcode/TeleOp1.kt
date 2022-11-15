package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.Arm
import org.firstinspires.ftc.teamcode.robot.Claw
import org.firstinspires.ftc.teamcode.robot.updateFieldCentric

@TeleOp(group = "comp-teleop")
class TeleOp1 : LinearOpMode() {
    private var prevGp1 = Gamepad()
    private var prevGp2 = Gamepad()

    override fun runOpMode() {
        val claw = Claw(hardwareMap, telemetry)
        val arm = Arm(hardwareMap, telemetry)
        val drive = SampleMecanumDrive(hardwareMap)
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        waitForStart()
        while (opModeIsActive()) {
            updateFieldCentric(drive, gamepad1)

            if (prevGp1.b && !gamepad1.b) claw.change()

            if (prevGp1.y && !gamepad1.y) arm.up()
            else if (prevGp1.a && !gamepad1.a) arm.down()

            if (gamepad1.dpad_up) arm.incUp()
            else if (gamepad1.dpad_down) arm.incDown()

            telemetry.addData("height", arm.height)
            telemetry.update()
            prevGp1.copy(gamepad1)
            prevGp2.copy(gamepad2)
        }
    }
}
