package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.Arm
import org.firstinspires.ftc.teamcode.robot.Claw
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline
import org.firstinspires.ftc.teamcode.vision.createWebcam

@Autonomous
class AutoLeft2 : LinearOpMode() {
    private lateinit var drive: SampleMecanumDrive
    private lateinit var trajs: Trajectories
    private lateinit var arm: Arm
    private lateinit var claw: Claw

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun runOpMode() {
        arm = Arm(hardwareMap, telemetry)
        claw = Claw(hardwareMap, telemetry, arm).apply { close() }
        drive = SampleMecanumDrive(hardwareMap).apply {
            poseEstimate = Pose2d(49.0, -6.0, Math.toRadians(55.0))
        }

        drive.followTrajectorySequenceAsync(
            drive.trajectorySequenceBuilder(Pose2d(49.0, -6.0, Math.toRadians(55.0)))

                .lineToSplineHeading(Pose2d(28.0, 0.0, 0.0))
                .back(24.0)
                .build()
        )

        val pipeline = AprilTagPipeline(tm).also { createWebcam(hardwareMap, telemetry, it) }
        waitForStart()

        val direction = pipeline.verdict

        while (opModeIsActive()) {
            arm.update()
            drive.update()
        }
    }
}
