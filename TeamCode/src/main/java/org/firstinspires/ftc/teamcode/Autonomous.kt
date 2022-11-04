package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.pipeline.AprilTagPipeline
import org.firstinspires.ftc.teamcode.pipeline.AprilTagPipeline.Tag
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation


fun Telemetry.putfs(fmt: String, vararg args: Any) {
    addLine(String.format(fmt, args))
    update()
}

@Autonomous
class AprilTagsOpMode : LinearOpMode() {

    override fun runOpMode() {
        telemetry.putfs("INIT.")
        val pipeline = AprilTagPipeline(telemetry)
        val cam = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get("Webcam 1") as WebcamName,
            hardwareMap.appContext.resources.getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.packageName
            )
        )
        cam.setPipeline(pipeline)
        cam.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() = cam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT)
            override fun onError(errorCode: Int) = telemetry.putfs("Camera error. Code: $errorCode")
        })
        FtcDashboard.getInstance().startCameraStream(cam, 30.0)

        val arm = Arm(hardwareMap)
        val claw = Claw(hardwareMap, telemetry)
        val drive = SampleMecanumDrive(hardwareMap)

        waitForStart()

        resetRuntime()
        telemetry.putfs("START. detected tag: ${pipeline.verdict.name}")


        // TODO autonomous

        val trajs = when (pipeline.verdict) {
            Tag.LEFT ->
                drive.trajectoryBuilder(Pose2d()).forward(28.0).build().let {
                    listOf(it, drive.trajectoryBuilder(it.end()).strafeLeft(23.0).build())
                }

            Tag.RIGHT ->
                drive.trajectoryBuilder(Pose2d()).forward(28.0).build().let {
                    listOf(it, drive.trajectoryBuilder(it.end()).strafeRight(23.0).build())
                }
            else -> listOf(
                drive.trajectoryBuilder(Pose2d()).forward(28.0).build()
            )
        }

        trajs.forEach { drive.followTrajectory(it) }
    }
}
