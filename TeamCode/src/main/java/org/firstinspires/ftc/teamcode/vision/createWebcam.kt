package org.firstinspires.ftc.teamcode.vision

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.subsystems.RobotConfig
import org.openftc.easyopencv.*

fun createWebcam(
    hardwareMap: HardwareMap, telemetry: Telemetry? = null, pipeline: OpenCvPipeline? = null
): OpenCvWebcam = OpenCvCameraFactory.getInstance().createWebcam(
    hardwareMap[RobotConfig.WEBCAM_1.s] as WebcamName,
    hardwareMap.appContext.resources.getIdentifier(
        "cameraMonitorViewId", "id", hardwareMap.appContext.packageName
    )
).apply {
    openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
        override fun onOpened() = startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT)
        override fun onError(errorCode: Int) {
            "Camera error. Code: $errorCode".let { telemetry?.addLine(it); println(it) }
        }
    })
    FtcDashboard.getInstance().startCameraStream(this, 30.0)
    pipeline?.let { setPipeline(it) }
}
