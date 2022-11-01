package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive

@TeleOp
class TestTeleOp : LinearOpMode() {

    private var prevGp = Gamepad()

    private var tuning = 'p'
    private var increment = 0.01

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

            if (prevGp.x && !gamepad1.x) tuning = when (tuning) {
                'p' -> 'i'
                'i' -> 'd'
                'd' -> 'p'
                else -> 'p'
            }

            if (gamepad1.y) when (tuning) {
                'i' -> Arm.kI += increment
                'd' -> Arm.kD += increment
                else -> Arm.kP += increment
            }
            else if (gamepad1.a) when (tuning) {
                'i' -> Arm.kI -= increment
                'd' -> Arm.kD -= increment
                else -> Arm.kP -= increment
            }



            if (prevGp.dpad_up && !gamepad1.dpad_up) arm.up()
            else if (prevGp.dpad_down && !gamepad1.dpad_up) arm.down()
            arm.update()


            telemetry.addLine("kP: ${Arm.kP}")
            telemetry.addLine("kI: ${Arm.kI}")
            telemetry.addLine("kD: ${Arm.kD}")
            telemetry.addLine("ref: ${arm.reference}")
            telemetry.addLine("enc: ${arm.motor.currentPosition}")

            telemetry.update()
            prevGp.copy(gamepad1)
        }
    }
}
