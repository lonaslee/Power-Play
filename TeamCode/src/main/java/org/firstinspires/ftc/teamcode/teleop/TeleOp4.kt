package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.*
import org.firstinspires.ftc.teamcode.vision.ConeDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.ConeDetectionPipeline.RED
import org.firstinspires.ftc.teamcode.vision.createWebcam
import org.openftc.easyopencv.OpenCvWebcam

@TeleOp
class TeleOp4 : OpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var drive: SampleMecanumDrive
    private lateinit var gamepads: Pair<GamepadExt, GamepadExt>
    private lateinit var webcam: OpenCvWebcam
    private val pipeline = ConeDetectionPipeline(RED)
    private val pickPID = PIDController(pP, pI, pD).apply { setPoint = 0.0 }

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun init() {
        arm = Arm(hardwareMap)
        claw = Claw(hardwareMap, arm = arm)
        drive = SampleMecanumDrive(hardwareMap)
        gamepads = GamepadExt(gamepad1) to GamepadExt(gamepad2)

        webcam = createWebcam(hardwareMap, pipeline = pipeline)
    }

    var picking = false
    override fun loop() {
        if (gamepads.first pressed Gamepad::left_bumper) picking = !picking
        if (picking && pipeline.detected) {
            pickPID.setPID(pP, pI, pD)
            gamepads.first.right_stick_x = -pickPID.calculate(pipeline.error).toFloat()
        }

        drive fieldcentricAccordingTo gamepads
        arm adjustAccordingTo gamepads
        claw changeAccordingTo gamepads

        gamepads.onEach { it.update() }
        tm.update()
    }

    companion object {
        @JvmField var pP = 0.002
        @JvmField var pI = 0.0
        @JvmField var pD = 0.0
    }
}
