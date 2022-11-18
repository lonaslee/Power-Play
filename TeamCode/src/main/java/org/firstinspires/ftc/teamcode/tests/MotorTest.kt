package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.Arm3
import org.firstinspires.ftc.teamcode.robot.Claw
import org.firstinspires.ftc.teamcode.robot.updateFieldCentric

@TeleOp
class MotorTest : LinearOpMode() {
    private var prevGp = Gamepad()

    private lateinit var arm: Arm3
    private lateinit var claw: Claw
    private lateinit var drive: SampleMecanumDrive

    override fun runOpMode() {
        val armLowMotor = hardwareMap["lowlift"] as DcMotorEx
        armLowMotor.direction = DcMotorSimple.Direction.REVERSE
        claw = Claw(hardwareMap, telemetry)

        waitForStart()
        if (isStopRequested) return
        while (opModeIsActive()) {
            updateFieldCentric(drive, gamepad1)

            if (prevGp.b && !gamepad1.b) claw.change()

            telemetry.addData("pos", armLowMotor.currentPosition)
            telemetry.update()

            prevGp.copy(gamepad1)
        }
    }
}
