package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad

import org.firstinspires.ftc.teamcode.robot.Arm
import org.firstinspires.ftc.teamcode.robot.Claw
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

@TeleOp(group = "comp-teleop")
class TeleOp4 : LinearOpMode() {
    private var prevGp = Gamepad()

    lateinit var leftFront: DcMotorEx
    lateinit var leftBack: DcMotorEx
    lateinit var rightFront: DcMotorEx
    lateinit var rightBack: DcMotorEx
    lateinit var motors: List<DcMotorEx>
    lateinit var claw: Claw
    lateinit var arm: Arm
    lateinit var imu: BNO055IMU

    override fun runOpMode() {
        claw = Claw(hardwareMap, telemetry)
        arm = Arm(hardwareMap, telemetry)
        imu = (hardwareMap["imu"] as BNO055IMU).apply { initialize(BNO055IMU.Parameters()) }
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        leftFront = hardwareMap["leftfront"] as DcMotorEx
        leftBack = hardwareMap["leftback"] as DcMotorEx
        rightFront = hardwareMap["rightfront"] as DcMotorEx
        rightBack = hardwareMap["rightback"] as DcMotorEx
        rightFront.direction = DcMotorSimple.Direction.REVERSE
        rightBack.direction = DcMotorSimple.Direction.REVERSE
        motors = listOf(leftFront, leftBack, rightFront, rightBack).onEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        waitForStart()
        while (opModeIsActive()) {
            with(motors) {
                val y = -gamepad1.left_stick_y.toDouble()
                val x = gamepad1.left_stick_x * 1.1
                val turn = gamepad1.right_stick_x.toDouble()

                val botHeading = -imu.angularOrientation.firstAngle.toDouble()
                val rotX = x * cos(botHeading) - y * sin(botHeading)
                val rotY = x * sin(botHeading) + y * cos(botHeading)

                max(abs(y) + abs(x) + abs(turn), 1.0).let { denom ->
                    get(0).power = ((rotY + rotX + turn) / denom) * .7
                    get(1).power = ((rotY - rotX + turn) / denom) * .7
                    get(2).power = ((rotY - rotX - turn) / denom) * .7
                    get(3).power = ((rotY + rotX - turn) / denom) * .7
                }
            }
            if (prevGp.b && !gamepad1.b) claw.change()

            if (prevGp.y && !gamepad1.y) arm.up()
            else if (prevGp.a && !gamepad1.a) arm.down()

            if (gamepad1.dpad_up) arm.incUp()
            else if (gamepad1.dpad_down) arm.incDown()

            telemetry.update()
            prevGp.copy(gamepad1)
        }
    }
}
