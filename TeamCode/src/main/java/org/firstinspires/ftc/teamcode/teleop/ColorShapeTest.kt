package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.RobotConfig
import org.firstinspires.ftc.teamcode.vision.ColorShapeDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.createWebcam
import org.openftc.easyopencv.OpenCvCameraRotation

@TeleOp
class ColorShapeTest : LinearOpMode() {
    override fun runOpMode() {
        val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val pipeline = ColorShapeDetectionPipeline(tm)
        val backWebcam = createWebcam(
            hardwareMap,
            RobotConfig.WEBCAM_2,
            pipeline,
            orientation = OpenCvCameraRotation.UPSIDE_DOWN
        )

        EventLoop(::opModeIsActive).apply {
            updates += { tm.update() }
        }.also {
            waitForStart()
            it.run()
        }
    }

    companion object {
        @JvmField var X = 120
        @JvmField var Y = 100
        @JvmField var W = 50
        @JvmField var H = 60
    }
}
