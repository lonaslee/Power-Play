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
 *
 * This does not start the stream to the driver station viewport, since this function could
 * be used to create multiple cameras.
 */
fun createWebcam(
    hardwareMap: HardwareMap,
    configName: RobotConfig,
    pipeline: OpenCvPipeline? = null,
    orientation: OpenCvCameraRotation = OpenCvCameraRotation.UPRIGHT,
) = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap[configName.s] as WebcamName).apply {
    openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
        override fun onOpened() = startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, orientation)
        override fun onError(errorCode: Int) = Unit
    })
    FtcDashboard.getInstance().startCameraStream(this, 30.0)
    pipeline?.let { setPipeline(it) }
}!!
