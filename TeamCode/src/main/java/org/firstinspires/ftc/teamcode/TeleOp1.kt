package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.Arm
import org.firstinspires.ftc.teamcode.robot.Claw

@TeleOp(group = "comp-teleop")
class TeleOp1 : LinearOpMode() {
    private var prevGp = Gamepad()

    override fun runOpMode() {
        val claw = Claw(hardwareMap, telemetry)
        val arm = Arm(hardwareMap, telemetry)
        val drive = SampleMecanumDrive(hardwareMap)
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        waitForStart()
        while (opModeIsActive()) {
            drive.setWeightedDrivePower(
                Pose2d(
                    -gamepad1.left_stick_y.toDouble() * 0.8,
                    -gamepad1.left_stick_x.toDouble() * 0.8,
                    -gamepad1.right_stick_x.toDouble() * 0.8
                )
            )
            drive.update()
            if (prevGp.b && !gamepad1.b) claw.change()

            if (prevGp.y && !gamepad1.y) arm.up()
            else if (prevGp.a && !gamepad1.a) arm.down()

            if (gamepad1.dpad_up) arm.incUp()
            else if (gamepad1.dpad_down) arm.incDown()

            telemetry.addData("height", arm.height)
            telemetry.update()

            prevGp.copy(gamepad1)
        }
    }
}
