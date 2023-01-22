package org.firstinspires.ftc.teamcode.vision

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.subsystems.RobotConfig
import org.openftc.easyopencv.*

const val CAMERA_WIDTH = 640
const val CAMERA_HEIGHT = 480

/**
 * Creates an [OpenCvWebcam] from the hardwaremap using the provided config's name. This
 * opens it and starts streaming using [CAMERA_WIDTH] and [CAMERA_HEIGHT], and streams it
 * to [FtcDashboard]. If a pipeline is given, the camera's pipeline will be set to it.
 */
fun createWebcam(
    hardwareMap: HardwareMap,
    configName: RobotConfig,
    telemetry: Telemetry? = null,
    pipeline: OpenCvPipeline? = null
): OpenCvWebcam = OpenCvCameraFactory.getInstance().createWebcam(
    hardwareMap[configName.s] as WebcamName, hardwareMap.appContext.resources.getIdentifier(
        "cameraMonitorViewId", "id", hardwareMap.appContext.packageName
    )
).apply {
    openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
        override fun onOpened() = startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT)

        override fun onError(errorCode: Int) {
            "Camera error. Code: $errorCode".let { telemetry?.addLine(it); println(it) }
        }
    })
    FtcDashboard.getInstance().startCameraStream(this, 30.0)
    pipeline?.let { setPipeline(it) }
}
