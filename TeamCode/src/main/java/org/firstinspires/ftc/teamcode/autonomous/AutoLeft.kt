package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline
import org.firstinspires.ftc.teamcode.robot.*
import org.firstinspires.ftc.teamcode.vision.createWebcam

@Autonomous
class AutoLeft : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var drive: SampleMecanumDrive
    private lateinit var trajs: LeftTraj

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    private val pipeline = AprilTagPipeline(tm)

    override fun runOpMode() {
        arm = Arm(hardwareMap)
        claw = Claw(hardwareMap, arm = arm).apply { close() }
        drive = SampleMecanumDrive(hardwareMap)
        trajs = LeftTraj(drive, arm, claw)

        createWebcam(hardwareMap, telemetry, pipeline)

        waitForStart()
        if (isStopRequested) return
        drive.followTrajectorySequenceAsync(trajs.byTag(pipeline.verdict))

        while (opModeIsActive()) {
            arm.update()
            drive.update()
        }
    }
}
