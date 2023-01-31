package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.apriltag.AprilTagDetectorJNI as TagDetector

class AprilTagPipeline(private val telemetry: Telemetry) : SignalSleevePipeline() {
    companion object {
        const val tagsize = 0.166
        const val fx = 578.272
        const val fy = 578.272
        const val cx = 402.145
        const val cy = 221.506
        val GREEN = Scalar(0.0, 255.0, 0.0, 255.0)

        const val LEFT_TAG_NUM = 809
        const val MIDDLE_TAG_NUM = 930
        const val RIGHT_TAG_NUM = 1436
    }

    private val tagdetectorPtr =
        TagDetector.createApriltagDetector(TagDetector.TagFamily.TAG_standard41h12.string, 3F, 3)
    private var greyscale = Mat()

    protected fun finalize() = TagDetector.releaseApriltagDetector(tagdetectorPtr)

    override var verdict = Tag.UNKNOWN
    override fun processFrame(input: Mat): Mat {
        try {
            Imgproc.cvtColor(input, greyscale, Imgproc.COLOR_RGBA2GRAY)
            val detections = TagDetector.runAprilTagDetectorSimple(
                tagdetectorPtr, greyscale, tagsize, fx, fy, cx, cy
            )
            if (detections.size == 0) telemetry.addLine("Not detected. FFFFFF")
            else with(detections[0]) {
                when (id) {
                    LEFT_TAG_NUM -> Tag.LEFT
                    MIDDLE_TAG_NUM -> Tag.MIDDLE
                    RIGHT_TAG_NUM -> Tag.RIGHT
                    else -> Tag.UNKNOWN
                }.let { tag ->
                    telemetry.addLine("Detected tag.\n  ID: ${id}\n  Pos: ${tag.name}")
                    verdict = tag
                }
                (corners[1].x - corners[0].x).let { radius ->
                    Imgproc.circle(input, center, radius.toInt(), GREEN, (radius / 8).toInt())
                    Imgproc.circle(
                        input, center, (radius / 20).toInt(), GREEN, (radius / 10).toInt()
                    )
                }
            }
        } catch (_: Exception) {

        }

        telemetry.addLine("Verdict: $verdict")
        telemetry.update()
        return input
    }
}
