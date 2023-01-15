package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline
import org.firstinspires.ftc.teamcode.vision.SignalSleevePipeline
import org.firstinspires.ftc.teamcode.vision.createWebcam

@Autonomous
class LeftAuto : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var drive: SampleMecanumDrive
    private lateinit var trajs: LeftTrajectory

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    private val pipeline: SignalSleevePipeline = AprilTagPipeline(tm)

    override fun runOpMode() {
        arm = Arm(hardwareMap)
        claw = Claw(hardwareMap).apply { state = Claw.CLOSED }
        drive =
            SampleMecanumDrive(
                hardwareMap
            )
        trajs = LeftTrajectory(drive, arm, claw)

        createWebcam(hardwareMap, telemetry, pipeline)

        waitForStart()
        if (isStopRequested) return
        drive.followTrajectorySequenceAsync(trajs.coneTraj)

        while (opModeIsActive()) {
            arm.update()
            drive.update()
        }
    }
}
