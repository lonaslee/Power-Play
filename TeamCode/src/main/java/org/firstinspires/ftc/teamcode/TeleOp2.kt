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
class TeleOp2 : LinearOpMode() {
    private var prevGp1 = Gamepad()
    private var prevGp2 = Gamepad()

    override fun runOpMode() {
        val claw = Claw(hardwareMap)
        val arm = Arm(hardwareMap, telemetry)
        val drive = SampleMecanumDrive(hardwareMap)
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        waitForStart()
        while (opModeIsActive()) {
            drive.setWeightedDrivePower(
                Pose2d(
                    -gamepad1.left_stick_y.toDouble() * 0.5,
                    -gamepad1.left_stick_x.toDouble() * 0.5,
                    -gamepad1.right_stick_x.toDouble() * 0.5
                )
            )
            drive.update()
            if (prevGp2.b && !gamepad2.b) claw.change()

            if (prevGp2.y && !gamepad2.y) arm.up()
            else if (prevGp2.a && !gamepad2.a) arm.down()

            prevGp1.copy(gamepad1)
            prevGp2.copy(gamepad2)
        }
    }
}
