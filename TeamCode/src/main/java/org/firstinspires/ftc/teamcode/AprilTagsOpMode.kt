package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.pipeline.AprilTagPipeline

fun Telemetry.putfs(fmt: String, vararg args: Any) {
    addLine(String.format(fmt, args))
    update()
}

@Autonomous()
class AprilTagsOpMode : LinearOpMode() {
    // TODO calibrate
    private val fx = 578.272
    private val fy = 578.272
    private val cx = 402.145
    private val cy = 221.506
    private val tagsize = 0.166

    override fun runOpMode() {
        val pipeline = AprilTagPipeline(telemetry)
    }
}
