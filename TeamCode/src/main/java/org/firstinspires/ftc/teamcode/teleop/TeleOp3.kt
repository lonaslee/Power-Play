package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.*
import org.firstinspires.ftc.teamcode.vision.ConeDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.ConeDetectionPipeline.RED
import org.firstinspires.ftc.teamcode.vision.createWebcam
import org.openftc.easyopencv.OpenCvWebcam

@TeleOp
class TeleOp3 : OpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var drive: SampleMecanumDrive
    private lateinit var gamepads: Pair<GamepadExt, GamepadExt>
    private lateinit var webcam: OpenCvWebcam
    private val pipeline = ConeDetectionPipeline(RED)
    private val pickPID = PIDController(pP, pI, pD).apply { setPoint = 0.0 }

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun init() {
        gamepads = GamepadExt(gamepad1) to GamepadExt(gamepad2)
        arm = Arm(hardwareMap)
        claw = Claw(hardwareMap)
        drive = SampleMecanumDrive(hardwareMap)

        webcam = createWebcam(hardwareMap, pipeline = pipeline)
    }

    private var picking = false
    override fun loop() {
        if (pressed(gamepads.first::right_bumper)) picking = !picking
        if (picking) {
            if (pipeline.detected) {
                pickPID.setPID(pP, pI, pD)
                gamepads.first.right_stick_x = -pickPID.calculate(pipeline.error).toFloat()
            }
            if (claw.state == Claw.States.CLOSED) picking = false
        }

        drive.update(gamepads)
        arm.update()

        gamepads.sync()
        tm.update()
    }

    companion object {
        @JvmField var pP = 0.002
        @JvmField var pI = 0.0
        @JvmField var pD = 0.0
    }
}
