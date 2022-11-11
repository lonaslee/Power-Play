package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

class FieldCentricDrive(hardwareMap: HardwareMap, private val telemetry: Telemetry) {
    private val leftFront = hardwareMap["leftfront"] as DcMotorEx
    private val leftBack = hardwareMap["leftback"] as DcMotorEx
    private val rightFront = (hardwareMap["rightfront"] as DcMotorEx).apply {
        direction = DcMotorSimple.Direction.REVERSE
    }
    private val rightBack = (hardwareMap["rightback"] as DcMotorEx).apply {
        direction = DcMotorSimple.Direction.REVERSE
    }
    private val motors = listOf(leftFront, leftBack, rightFront, rightBack).onEach {
        it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    private val imu = (hardwareMap["imu"] as BNO055IMU).apply { initialize(BNO055IMU.Parameters()) }

    var speed = 0.6

    fun update(gamepad1: Gamepad) {
        val y = -gamepad1.left_stick_y.toDouble()
        val x = gamepad1.left_stick_x * 1.1
        val turn = gamepad1.right_stick_x.toDouble()

        val botHeading = -imu.angularOrientation.firstAngle.toDouble()
        val rotX = x * cos(botHeading) - y * sin(botHeading)
        val rotY = x * sin(botHeading) + y * cos(botHeading)

        max(abs(y) + abs(x) + abs(turn), 1.0).let { denom ->
            leftFront.power = ((rotY + rotX + turn) / denom) * speed
            leftBack.power = ((rotY - rotX + turn) / denom) * speed
            rightFront.power = ((rotY - rotX - turn) / denom) * speed
            rightBack.power = ((rotY + rotX - turn) / denom) * speed
        }
    }
}