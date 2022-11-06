package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive

@TeleOp
class MotorTest : LinearOpMode() {

    private var lastGamepad = Gamepad()
    private fun bpressed() = lastGamepad.b && !gamepad1.b
    private fun xpressed() = lastGamepad.x && !gamepad1.x

    override fun runOpMode() {
//        val arm = Arm(hardwareMap, telemetry)
        val armmotor = hardwareMap.get("lowlift") as DcMotorEx
        armmotor.direction = DcMotorSimple.Direction.REVERSE
        val clawservo = hardwareMap.get("claw") as ServoImplEx
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

            telemetry.addLine("armEnc: ${armmotor.currentPosition}")
            telemetry.update()

            lastGamepad.copy(gamepad1)


//            arm.update()
        }
    }
}
