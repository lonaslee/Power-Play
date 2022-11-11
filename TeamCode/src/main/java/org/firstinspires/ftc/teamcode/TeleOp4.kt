package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad

import org.firstinspires.ftc.teamcode.robot.Arm3
import org.firstinspires.ftc.teamcode.robot.Claw
import org.firstinspires.ftc.teamcode.robot.FieldCentricDrive

@TeleOp(group = "comp-teleop")
class TeleOp4 : LinearOpMode() {
    private var prevGp = Gamepad()

    lateinit var claw: Claw
    lateinit var arm: Arm3
    lateinit var drive: FieldCentricDrive

    override fun runOpMode() {
        claw = Claw(hardwareMap, telemetry)
        arm = Arm3(hardwareMap, telemetry)
        drive = FieldCentricDrive(hardwareMap, telemetry)

        waitForStart()
        if (isStopRequested) return
        while (opModeIsActive()) {
            drive.update(gamepad1)

            if (prevGp.b && !gamepad1.b) claw.change()

            if (prevGp.y && !gamepad1.y) arm.up()
            else if (prevGp.a && !gamepad1.a) arm.down()
            arm.update()

            telemetry.update()
            prevGp.copy(gamepad1)
        }
    }
}
