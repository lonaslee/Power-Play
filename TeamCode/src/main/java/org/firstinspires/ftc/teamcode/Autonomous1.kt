package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.pipeline.AprilTagPipeline
import org.firstinspires.ftc.teamcode.pipeline.AprilTagPipeline.Tag
import org.firstinspires.ftc.teamcode.robot.Arm
import org.firstinspires.ftc.teamcode.robot.Claw
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
    override fun runOpMode() {
        telemetry.putfs("INIT.")
        val pipeline = AprilTagPipeline(telemetry).also {
            OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap["Webcam 1"] as WebcamName,
                hardwareMap.appContext.resources.getIdentifier(
                    "cameraMonitorViewId", "id", hardwareMap.appContext.packageName
                )
            ).apply {
                setPipeline(it)
                openCameraDeviceAsync(object : AsyncCameraOpenListener {
                    override fun onOpened() = startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT)

                    override fun onError(errorCode: Int) =
                        telemetry.putfs("Camera error. Code: $errorCode")
                })
                FtcDashboard.getInstance().startCameraStream(this, 30.0)
            }
        }
        val arm = Arm(hardwareMap, telemetry)
        val claw = Claw(hardwareMap, telemetry)
        val drive = SampleMecanumDrive(hardwareMap)

        waitForStart()
        resetRuntime()
        telemetry.putfs("START. detected tag: ${pipeline.verdict.name}")

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
