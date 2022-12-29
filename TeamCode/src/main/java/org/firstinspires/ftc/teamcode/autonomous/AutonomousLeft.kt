package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.autonomous.LeftTraj.Companion.rad
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.subsystems.Arm
import org.firstinspires.ftc.teamcode.robot.subsystems.Claw
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline
import org.firstinspires.ftc.teamcode.vision.createWebcam

@Autonomous
class AutonomousLeft : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var drive: SampleMecanumDrive

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    private val pipeline = AprilTagPipeline(tm)

    override fun runOpMode() {
        arm = Arm(hardwareMap)
        claw = Claw(hardwareMap).apply { close() }
        drive = SampleMecanumDrive(hardwareMap)
        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(Pose2d())
            .addTemporalMarker { arm.state = Arm.States.MID }
            .splineToLinearHeading(LeftTraj.Pose2d(31, -3, -40), 0.rad)
            .waitSeconds(0.2)
            .addTemporalMarker { claw.open() }
            .waitSeconds(0.2)
            .UNSTABLE_addTemporalMarkerOffset(0.3) { arm.state = Arm.States.STACK }
            .lineToLinearHeading(LeftTraj.Pose2d(28, 0, 90))
            .addTemporalMarker { arm.state = Arm.States.GROUND }
            .build())
        createWebcam(hardwareMap, telemetry, pipeline)

        waitForStart()
        if (isStopRequested) return
        val dir = pipeline.verdict

        while (opModeIsActive()) {
            arm.update()
            drive.update()

            if (!drive.isBusy) {
                break
            }
        }
        when (dir) {
            AprilTagPipeline.Tag.LEFT -> drive.followTrajectory(
                drive.trajectoryBuilder(LeftTraj.Pose2d(55, 10, 90))
                    .forward(28.0)
                    .build()
            )
            AprilTagPipeline.Tag.RIGHT -> drive.followTrajectory(
                drive.trajectoryBuilder(LeftTraj.Pose2d(55, 10, 90))
                    .back(28.0)
                    .build()
            )
            else -> {}
        }
        while (opModeIsActive()) {
            drive.update()
            arm.update()
        }
    }
}
