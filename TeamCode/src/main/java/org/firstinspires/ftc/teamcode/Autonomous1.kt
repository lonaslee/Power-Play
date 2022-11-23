package org.firstinspires.ftc.teamcode

import android.annotation.SuppressLint
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.pipeline.AprilTagPipeline
import org.firstinspires.ftc.teamcode.pipeline.AprilTagPipeline.Tag
import org.firstinspires.ftc.teamcode.robot.Arm3
import org.firstinspires.ftc.teamcode.robot.Claw
import org.firstinspires.ftc.teamcode.robot.Config
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation


fun Telemetry.putfs(fmt: String, vararg args: Any) = String.format(fmt, args).let { msg ->
    println(msg)
    addLine(msg)
    update()
    Unit
}

@Autonomous(group = "comp-auto")
class Autonomous1 : LinearOpMode() {
    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry);

    override fun runOpMode() {
        telemetry.putfs("INIT.")
        val pipeline = AprilTagPipeline(tm)
        OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap[Config.WEBCAM_1.s] as WebcamName,
            hardwareMap.appContext.resources.getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.packageName
            )
        ).apply {
            setPipeline(pipeline)
            openCameraDeviceAsync(object : AsyncCameraOpenListener {
                override fun onOpened() = startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT)

                override fun onError(errorCode: Int) = tm.putfs("Camera error. Code: $errorCode")
            })
            FtcDashboard.getInstance().startCameraStream(this, 30.0)
        }

        val arm = Arm3(hardwareMap, tm)
        val claw = Claw(hardwareMap, tm)
        val drive = SampleMecanumDrive(hardwareMap)

        waitForStart()
        resetRuntime()
        tm.putfs("START. detected tag: ${pipeline.verdict.name}")

        // TODO autonomous

        when (pipeline.verdict) {
            Tag.LEFT -> drive.trajectoryBuilder(Pose2d()).forward(28.0).build().let {
                listOf(it, drive.trajectoryBuilder(it.end()).strafeLeft(23.0).build())
            }
            Tag.RIGHT -> drive.trajectoryBuilder(Pose2d()).forward(28.0).build().let {
                listOf(it, drive.trajectoryBuilder(it.end()).strafeRight(23.0).build())
            }
            else -> listOf(drive.trajectoryBuilder(Pose2d()).forward(28.0).build())
        }.forEach { drive.followTrajectory(it) }
    }
}
