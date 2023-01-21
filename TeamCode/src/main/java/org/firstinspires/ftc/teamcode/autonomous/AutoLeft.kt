package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Claw.States.CLOSED
import org.firstinspires.ftc.teamcode.subsystems.RobotConfig
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline
import org.firstinspires.ftc.teamcode.vision.SignalSleevePipeline
import org.firstinspires.ftc.teamcode.vision.createWebcam

@Autonomous
class AutoLeft : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var drive: SampleMecanumDrive
    private lateinit var trajs: LeftTraj

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    private val pipeline: SignalSleevePipeline = AprilTagPipeline(tm)

    override fun runOpMode() {
        arm = Arm(hardwareMap)
        claw = Claw(hardwareMap)
        drive = SampleMecanumDrive(hardwareMap)
        trajs = LeftTraj(drive, arm, claw)

        createWebcam(hardwareMap, RobotConfig.WEBCAM_1, telemetry, pipeline)

        waitForStart()
        if (isStopRequested) return
        claw.state = CLOSED
        drive.followTrajectorySequenceAsync(trajs.byTag(pipeline.verdict))
        sleep(500)

        while (opModeIsActive()) {
            arm.update()
            drive.update()
            if (!drive.isBusy) break
        }
    }
}
