package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.pipeline.AprilTagPipeline
import org.firstinspires.ftc.teamcode.robot.Arm3
import org.firstinspires.ftc.teamcode.robot.Claw
import org.firstinspires.ftc.teamcode.robot.Config
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

@Autonomous
class AutoLeft : OpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm3
    private lateinit var drive: SampleMecanumDrive
    private lateinit var trajs: LeftTrajectories

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    private var stage = 0

    override fun loop() {
        if (stage == 0) {
            tag = pipeline.verdict
            repeat(2) { arm.up() }
            stage++
        }
        if (!drive.isBusy) {
            claw.open()
//            drive.followTrajectorySequenceAsync(trajs.toStack)
            repeat(2) { arm.down() }
            stage++
        }

        arm.update()
        drive.update()
    }

    private lateinit var pipeline: AprilTagPipeline
    private lateinit var tag: AprilTagPipeline.Tag
    override fun init() {
        claw = Claw(hardwareMap, tm)
        arm = Arm3(hardwareMap, tm)
        drive = SampleMecanumDrive(hardwareMap)
        trajs = LeftTrajectories(drive)
        drive.followTrajectoryAsync(trajs.toMid)
        claw.close()

        pipeline = AprilTagPipeline(tm)
        OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap[Config.WEBCAM_1.s] as WebcamName,
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
