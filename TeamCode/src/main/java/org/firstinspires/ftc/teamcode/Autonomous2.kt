package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.pipeline.AprilTagPipeline
import org.firstinspires.ftc.teamcode.robot.Arm3
import org.firstinspires.ftc.teamcode.robot.Claw
import org.firstinspires.ftc.teamcode.robot.Config
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

@Autonomous
class Autonomous2 : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm3
    private lateinit var drive: SampleMecanumDrive

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun runOpMode() {
        initPhase()
        val toMid = drive.trajectoryBuilder(Pose2d(-36.0, -60.0))
            .lineToSplineHeading(Pose2d(-40.0, -40.0, -150.0)).build()

    }

    private lateinit var tag: AprilTagPipeline.Tag
    private fun initPhase() {
        claw = Claw(hardwareMap, tm)
        arm = Arm3(hardwareMap, tm)
        drive = SampleMecanumDrive(hardwareMap)

        val pipeline = AprilTagPipeline(tm)
        OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap[Config.WEBCAM_1.s] as WebcamName,
            hardwareMap.appContext.resources.getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.packageName
            )
        ).apply {
            setPipeline(pipeline)
            openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
                override fun onOpened() = startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT)
                override fun onError(errorCode: Int) {
                    tm.addLine("Camera error. Code: $errorCode")
                }
            })
            FtcDashboard.getInstance().startCameraStream(this, 30.0)
        }
        waitForStart()
        resetRuntime()
        tag = pipeline.verdict
    }
}
