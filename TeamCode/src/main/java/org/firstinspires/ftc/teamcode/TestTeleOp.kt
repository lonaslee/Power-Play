package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive

@TeleOp
@Config
class TestTeleOp : LinearOpMode() {

    private var prevGp = Gamepad()

    override fun runOpMode() {
        val claw = Claw(hardwareMap, telemetry)
        val arm = Arm(hardwareMap, telemetry)
        val drive = SampleMecanumDrive(hardwareMap)
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        println("INIT finish")
        waitForStart()

        while (opModeIsActive()) {
            drive.setWeightedDrivePower(
                Pose2d(
                    -gamepad1.left_stick_y.toDouble(),
                    -gamepad1.left_stick_x.toDouble(),
                    -gamepad1.right_stick_x.toDouble()
                )
            )
            drive.update()

            if (prevGp.b && !gamepad1.b) claw.change()

            if (prevGp.y && !gamepad1.y) arm.up()
            else if (prevGp.a && !gamepad1.a) arm.down()
            arm.update()


            telemetry.addData("ref", arm.reference)
            telemetry.addData("enc", arm.motor.currentPosition)
            telemetry.update()
            prevGp.copy(gamepad1)
        }
    }
}
