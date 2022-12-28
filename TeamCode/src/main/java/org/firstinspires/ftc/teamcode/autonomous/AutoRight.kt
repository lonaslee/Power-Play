package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.Arm
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline
import org.firstinspires.ftc.teamcode.robot.Claw
import org.firstinspires.ftc.teamcode.robot.RobotConfig
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

@Autonomous
class AutoRight : OpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var drive: SampleMecanumDrive
    private lateinit var trajs: Trajectories

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    private var stage = 0

    override fun loop() {
        if (stage == 0) {
            arm.state = Arm.Height.MID
            stage++
        }
        if (!drive.isBusy) {
            claw.open()
            arm.state = Arm.Height.GROUND
            stage++
        }

        arm.update()
        drive.update()
    }

    private lateinit var tag: AprilTagPipeline.Tag
    override fun init() {
        claw = Claw(hardwareMap, tm)
        arm = Arm(hardwareMap, tm)
        drive = SampleMecanumDrive(hardwareMap)
        trajs = Trajectories(drive, arm, claw)
        drive.followTrajectorySequenceAsync(trajs.left)
        claw.close()

        val pipeline = AprilTagPipeline(tm)
        OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap[RobotConfig.WEBCAM_1.s] as WebcamName,
            hardwareMap.appContext.resources.getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.packageName
            )
        ).apply {
            setPipeline(pipeline)
            openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
                override fun onOpened() = startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT)
                override fun onError(errorCode: Int) {
                    tm.addLine("Camera error. Code: $errorCode")
                }
            })
            FtcDashboard.getInstance().startCameraStream(this, 30.0)
        }
    }
}
