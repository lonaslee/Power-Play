package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive

import org.firstinspires.ftc.teamcode.robot.Arm3
import org.firstinspires.ftc.teamcode.robot.Claw
import org.firstinspires.ftc.teamcode.robot.updateFieldCentric

@TeleOp(group = "comp-teleop")
class TeleOp4 : LinearOpMode() {
    private var prevGp1 = Gamepad()
    private var prevGp2 = Gamepad()

    lateinit var claw: Claw
    lateinit var arm: Arm3
    lateinit var drive: SampleMecanumDrive

    override fun runOpMode() {
        claw = Claw(hardwareMap, telemetry)
        arm = Arm3(hardwareMap, telemetry)
        drive = SampleMecanumDrive(hardwareMap)

        waitForStart()
        if (isStopRequested) return
        while (opModeIsActive()) {
            updateFieldCentric(drive, gamepad1)

            if (prevGp1.b && !gamepad1.b) claw.change()

            if (prevGp1.y && !gamepad1.y) arm.up()
            else if (prevGp1.a && !gamepad1.a) arm.down()
            arm.update()

            telemetry.update()
            prevGp1.copy(gamepad1)
            prevGp2.copy(gamepad2)
        }
    }
}
