package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.autonomous.Trajectories
import org.firstinspires.ftc.teamcode.autonomous.Trajectories.byTag
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Arm4
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.RobotConfig
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline
import org.firstinspires.ftc.teamcode.vision.createWebcam
import org.openftc.easyopencv.OpenCvCameraRotation

@Autonomous
class AutoLeft : LinearOpMode() {
    override fun runOpMode() {
        val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val pipeline = AprilTagPipeline(tm)
        val webcam = createWebcam(
            hardwareMap,
            RobotConfig.WEBCAM_2,
            pipeline = pipeline,
            orientation = OpenCvCameraRotation.UPSIDE_DOWN
        )

        val drive = SampleMecanumDrive(hardwareMap)
        val arm = object : Arm4(hardwareMap, tm) {
            override var state: Int
                get() = super.state
                set(_) {}
        }
        val claw = object : Claw(hardwareMap) {
            override var state: Double
                get() = super.state
                set(_) {}
        }
        val trajs = Trajectories.generateLeft(drive, arm, claw)

        waitForStart()
        drive.followTrajectorySequenceAsync(trajs.byTag(pipeline.verdict))

        while (opModeIsActive()) {
            drive.update()
            arm.update()
        }
    }
}
