package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp
class MotorTest : LinearOpMode() {
    override fun runOpMode() {
        val motor = hardwareMap.get("lift_motor") as DcMotor
        waitForStart()

        while (opModeIsActive()) {
            motor.power = -gamepad1.left_stick_y.toDouble()
            sleep(20)
        }
    }
}