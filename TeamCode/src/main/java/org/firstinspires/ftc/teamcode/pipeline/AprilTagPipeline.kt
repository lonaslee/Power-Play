package org.firstinspires.ftc.teamcode.pipeline

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import org.openftc.apriltag.AprilTagDetectorJNI as TagDetector

class AprilTagPipeline(private val telemetry: Telemetry) : OpenCvPipeline() {
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


    private val tagdetectorPtr = TagDetector.createApriltagDetector(Tag.tagFamily, 3F, 3)
    private var greyscale = Mat()

    protected fun finalize() = TagDetector.releaseApriltagDetector(tagdetectorPtr)

    var verdict = Tag.UNKNOWN
    override fun processFrame(input: Mat): Mat {
        Imgproc.cvtColor(input, greyscale, Imgproc.COLOR_RGBA2GRAY)
        val detections = TagDetector.runAprilTagDetectorSimple(
            tagdetectorPtr, greyscale, tagsize, fx, fy, cx, cy
        )
        if (detections.size == 0) telemetry.addLine("Not detected. FFFFFF")
        else with(detections[0]) {
            Tag.getTag(id).let { tag ->
                telemetry.addLine("Detected tag.\n  ID: ${tag.id}\n  Pos: ${tag.name}")
                verdict = tag
            }
            (corners[1].x - corners[0].x).let { radius ->
                Imgproc.circle(input, center, radius.toInt(), GREEN, (radius / 8).toInt())
                Imgproc.circle(
                    input, center, (radius / 20).toInt(), GREEN, (radius / 10).toInt()
                )
            }
        }
        telemetry.addLine("Verdict: $verdict")
        telemetry.update()
        return input
    }
}
