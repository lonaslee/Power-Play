package org.firstinspires.ftc.teamcode.pipeline

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import org.openftc.apriltag.AprilTagDetectorJNI as TagDetector

class AprilTagPipeline(val telemetry: Telemetry) : OpenCvPipeline() {
    companion object {
        const val tagsize = 0.166
        const val fx = 578.272
        const val fy = 578.272
        const val cx = 402.145
        const val cy = 221.506
        val GREEN = Scalar(0.0, 255.0, 0.0, 255.0)
    }

    enum class Tag(val id: Int) {
        LEFT(809), MIDDLE(930), RIGHT(1436), UNKNOWN(-1);

        val pos: String = name

        companion object {
            const val tagFamily = "tagStandard41h12"
            fun getTag(id: Int) = when (id) {
                809 -> LEFT
                930 -> MIDDLE
                1436 -> RIGHT
                else -> UNKNOWN
            }
        }
    }


    private val tagDetector_ptr = TagDetector.createApriltagDetector(Tag.tagFamily, 3F, 3)
    private var greyscale = Mat()

    protected fun finalize() = TagDetector.releaseApriltagDetector(tagDetector_ptr)

    var verdict = Tag.UNKNOWN
    override fun processFrame(input: Mat): Mat {
        Imgproc.cvtColor(input, greyscale, Imgproc.COLOR_RGBA2GRAY)
        val (detection, tag) = TagDetector.runAprilTagDetectorSimple(
            tagDetector_ptr, greyscale, tagsize, fx, fy, cx, cy
        ).takeUnless { it.size == 0 }?.let { Pair(it[0], Tag.getTag(it[0].id)) } ?: Pair(null, null)

        if (detection == null || tag == null) {
            telemetry.addLine("Not detected. F")
        } else {
            telemetry.addLine("Detected tag. \n  ID: ${tag.id}\n  Pos: ${tag.pos}")
            verdict = tag

            val rad = detection.corners[1].x - detection.corners[0].x
            Imgproc.circle(input, detection.center, rad.toInt(), GREEN, (rad / 8).toInt())
            Imgproc.circle(input, detection.center, (rad / 20).toInt(), GREEN, (rad/10).toInt())
        }
        telemetry.addLine("Cur/Last: $verdict")
        telemetry.update()
        return input
    }
}
