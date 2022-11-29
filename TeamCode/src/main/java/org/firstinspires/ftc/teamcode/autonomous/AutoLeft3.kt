package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.Arm
import org.firstinspires.ftc.teamcode.robot.Claw
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline
import org.firstinspires.ftc.teamcode.vision.createWebcam

@Autonomous
class AutoLeft3 : OpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var drive: SampleMecanumDrive
    private lateinit var trajs: LeftTraj

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    private val pipeline = AprilTagPipeline(tm)

    override fun init() {
        claw = Claw(hardwareMap, telemetry).apply { close() }
        arm = Arm(hardwareMap, telemetry)
        drive = SampleMecanumDrive(hardwareMap)
        trajs = LeftTraj(drive, arm, claw)
        createWebcam(hardwareMap, telemetry, pipeline)
    }

    override fun loop() {
        drive.update()
        arm.update()
    }
}