package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline.Tag
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.openftc.easyopencv.OpenCvCamera
import kotlin.run


@Autonomous()
class AprilTagsOpMode : LinearOpMode() {
    // TODO calibrate
    val fx = 578.272
    val fy = 578.272
    val cx = 402.145
    val cy = 221.506

    val tagsize = 0.166

    override fun runOpMode() {
        fun Telemetry.puts(ln: String) {
            telemetry.addLine(ln)
            telemetry.update()
        }

        val pipeline = AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy)
        pipeline.init_cam(hardwareMap, telemetry)

        while (!isStarted && !isStopRequested)
            pipeline.init_loop()
        if (isStopRequested) return

        // TODO Autonomous run yadayadayada

        when (pipeline.detection) {
            Tag.LEFT -> run {

            }
            Tag.MIDDLE, null -> run {

            }
            Tag.RIGHT -> run {

            }
        }
    }
}
