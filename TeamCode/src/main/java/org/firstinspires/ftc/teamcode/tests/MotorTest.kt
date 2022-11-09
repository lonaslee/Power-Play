package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.Arm2
import org.firstinspires.ftc.teamcode.robot.Claw

@TeleOp
class MotorTest : LinearOpMode() {
    private var prevGp = Gamepad()

    lateinit var arm: Arm2
    lateinit var claw: Claw
    lateinit var drive: SampleMecanumDrive

    override fun runOpMode() {
        arm = Arm2(hardwareMap, telemetry)
        claw = Claw(hardwareMap, telemetry)
        drive =
            SampleMecanumDrive(hardwareMap).apply { setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER) }
        waitForStart()
        if (isStopRequested) return

        while (opModeIsActive()) {
            updateDrive()
            updateClaw()
            updateArm()
            telemetry.update()
            prevGp.copy(gamepad1)
        }
    }

    private fun updateDrive() {
        drive.setWeightedDrivePower(
            Pose2d(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble()
            )
        )
        drive.update()
    }

    private fun updateClaw() {
        if (prevGp.b && !gamepad1.b) claw.change()
    }

    private fun updateArm() {
        if (prevGp.y && !gamepad1.y) arm.up()
        else if (prevGp.a && !gamepad1.a) arm.down()
        arm.update()
    }
}
