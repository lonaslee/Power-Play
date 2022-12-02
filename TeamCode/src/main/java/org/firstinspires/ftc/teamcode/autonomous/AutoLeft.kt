package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline
import org.firstinspires.ftc.teamcode.robot.*
import org.firstinspires.ftc.teamcode.vision.createWebcam
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

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
        val dir = pipeline.verdict

        while (opModeIsActive()) {
            arm.update()
            drive.update()

            if (!drive.isBusy) {
                when (dir) {
                    AprilTagPipeline.Tag.LEFT  -> drive.followTrajectory(
                        drive.trajectoryBuilder(LeftTraj.endPose).forward(28.0).build()
                    )
                    AprilTagPipeline.Tag.RIGHT -> drive.followTrajectory(
                        drive.trajectoryBuilder(LeftTraj.endPose).back(28.0).build()
                    )
                    else                       -> {}
                }
            }
        }
    }
}
