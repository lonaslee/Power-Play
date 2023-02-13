package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.autonomous.LeftTrajectory.Companion.rad
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline
import org.firstinspires.ftc.teamcode.vision.SignalSleevePipeline
import org.firstinspires.ftc.teamcode.vision.createWebcam

@Autonomous
@Config
class LeftAuto1 : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm3
    private lateinit var drive: SampleMecanumDrive

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    private val pipeline: SignalSleevePipeline = AprilTagPipeline(tm)

    override fun runOpMode() {
        arm = Arm3(hardwareMap, tm)
        claw = Claw(hardwareMap)
        drive = SampleMecanumDrive(hardwareMap)

        val camera = createWebcam(hardwareMap, RobotConfig.WEBCAM_2, pipeline)
        waitForStart()
        if (isStopRequested) return
        camera.closeCameraDeviceAsync {}

        val verdict = pipeline.verdict

//        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(LeftTrajectory.startPose)
//            .setReversed(true)
//            .addTemporalMarker { arm.state = Arm.BACKHIGH }
//            .setAccelConstraint { _, _, _, _ -> LeftTrajectory.acc }
//            .splineTo(Vector2d(X, Y), H.rad)
//            .waitSeconds(LeftTrajectory.dropWait)
//            .addTemporalMarker { claw.state = Claw.OPENED }
//            .waitSeconds(LeftTrajectory.t)
//            .addTemporalMarker { arm.state = Arm.GROUND }
//            .waitSeconds(LeftTrajectory.d)
//            .setReversed(false)
//            .splineTo(Vector2d(-36.0, -10.5), 0.rad)
//            .build()!!
//        )

        drive.followTrajectory(drive.trajectoryBuilder(Pose2d())
            .back(X)
            .build())

        drive.poseEstimate = Pose2d()

        when (verdict) {
            SignalSleevePipeline.Tag.LEFT -> drive.followTrajectory(
                drive.trajectoryBuilder(Pose2d())
                    .strafeRight(Y)
                    .build()
            )
            SignalSleevePipeline.Tag.RIGHT -> drive.followTrajectory(
                drive.trajectoryBuilder(Pose2d())
                    .strafeLeft(Y)
                    .build()
            )
            else -> {}
        }
    }

    companion object {
        @JvmField var X = 24.0
        @JvmField var Y = 24.0
        @JvmField var H = 45
    }
}
