package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

/**
 * Updates a drive's powers to field centric values based on gamepad input and imu angle.
 * This depends on a modification to the SampleMecanumDrive class, making the 'imu' field public.
 *
 * @see SampleMecanumDrive.setMotorPowers
 * @see SampleMecanumDrive.imu
 */
fun updateFieldCentric(
    drive: SampleMecanumDrive, gamepad1: Gamepad, speed: Double = 0.6
) {
    val y = -gamepad1.left_stick_y.toDouble()
    val x = gamepad1.left_stick_x * 1.1
    val turn = gamepad1.right_stick_x.toDouble()

    val botHeading = -drive.imu.angularOrientation.firstAngle.toDouble()
    val rotX = x * cos(botHeading) - y * sin(botHeading)
    val rotY = x * sin(botHeading) + y * cos(botHeading)

    val denom = max(abs(y) + abs(x) + abs(turn), 1.0)
    drive.setMotorPowers(
        (rotY + rotX + turn) / denom * speed,
        (rotY - rotX + turn) / denom * speed,
        (rotY - rotX - turn) / denom * speed,
        (rotY + rotX - turn) / denom * speed
    )
    drive.update()
}
