package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline
import org.firstinspires.ftc.teamcode.robot.*
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

@Autonomous
class AutoLeft : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var drive: SampleMecanumDrive
    private lateinit var trajs: Trajectories

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    private val pipeline = AprilTagPipeline(tm)

    override fun runOpMode() {
        initPhase()

        waitForStart()
        if (isStopRequested) return

        while (opModeIsActive()) {
            arm.update()
            drive.update()
        }
    }

    private fun initPhase() {
        claw = Claw(hardwareMap, telemetry).apply { close() }
        arm = Arm(hardwareMap, telemetry)
        drive = SampleMecanumDrive(hardwareMap).apply {
            poseEstimate = Trajectories.leftStartPos
            trajs = Trajectories(this, arm, claw)
            followTrajectorySequenceAsync(trajs.left)
        }

        OpenCvCameraFactory.getInstance()
            .createWebcam(
                hardwareMap[RobotConfig.WEBCAM_1.s] as WebcamName,
                hardwareMap.appContext.resources.getIdentifier(
                    "cameraMonitorViewId", "id", hardwareMap.appContext.packageName
                )
            )
            .apply {
                setPipeline(pipeline)
                openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
                    override fun onOpened() = startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT)
                    override fun onError(errorCode: Int) {
                        tm.addLine("Camera error. Code: $errorCode")
                    }
                })
                FtcDashboard.getInstance()
                    .startCameraStream(this, 30.0)
            }
    }
}
