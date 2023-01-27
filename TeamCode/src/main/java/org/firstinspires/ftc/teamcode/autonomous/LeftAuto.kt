package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.teleop.EventLoop
import org.firstinspires.ftc.teamcode.vision.ColorShapeDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.SignalSleevePipeline
import org.firstinspires.ftc.teamcode.vision.createWebcam

@Autonomous
class LeftAuto : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm3
    private lateinit var drive: SampleMecanumDrive
    private lateinit var trajs: LeftTrajectory

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    private val pipeline: SignalSleevePipeline = ColorShapeDetectionPipeline(tm)

    override fun runOpMode() {
        arm = Arm3(hardwareMap)
        claw = Claw(hardwareMap).apply { state = Claw.CLOSED }
        drive = SampleMecanumDrive(hardwareMap).apply { poseEstimate = LeftTrajectory.startPose }
        trajs = LeftTrajectory(drive, arm, claw)

        val camera = createWebcam(hardwareMap, RobotConfig.WEBCAM_2, pipeline)

        EventLoop(::opModeIsActive, tm).apply {
            updates += listOf(arm::update, drive::update)
        }.also {
            waitForStart()
            if (isStopRequested) return
            camera.closeCameraDeviceAsync {}

            drive.followTrajectorySequenceAsync(trajs.byTag(pipeline.verdict))
            it.run()
        }
        DriveExt.PoseStorage.pose = drive.poseEstimate
    }
}
