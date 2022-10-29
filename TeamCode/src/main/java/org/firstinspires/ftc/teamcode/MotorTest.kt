package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive

@TeleOp
class MotorTest : LinearOpMode() {

    private var lastGamepad = Gamepad()
    private fun bpressed() = lastGamepad.b && !gamepad1.b
    private fun xpressed() = lastGamepad.x && !gamepad1.x

    override fun runOpMode() {
        val arm = Arm(hardwareMap, telemetry)
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

            if (lastGamepad.b && !gamepad1.b)
                clawservo.position = if (clawservo.position < 0.55) 0.64 else 0.5


            lastGamepad.copy(gamepad1)

            if (lastGamepad.y && !gamepad1.y) arm.reference = 10.0
            else if (lastGamepad.a && !gamepad1.a) arm.reference = 0.0

//            arm.update()
        }
    }
}
