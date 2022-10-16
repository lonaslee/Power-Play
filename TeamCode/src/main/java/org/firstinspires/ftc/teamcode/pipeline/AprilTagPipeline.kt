package org.firstinspires.ftc.teamcode.pipeline

import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc
import org.openftc.apriltag.AprilTagDetectorJNI
import org.openftc.easyopencv.OpenCvPipeline

class AprilTagDetector : OpenCvPipeline() {


    enum class Tag(val id: Int) {
        LEFT(809), MIDDLE(930), RIGHT(1436);

        val pos: String = name

        companion object {
            const val tagFamily = "tagStandard41h12"
            fun getTag(id: Int): Tag {
                return when (id) {
                    809 -> LEFT
                    930 -> MIDDLE
                    1436 -> RIGHT
                    else -> throw IllegalArgumentException("Invalid getTag() ID.")
                }
            }
        }
    }

    private lateinit var greyscale: Mat
    private val a: Long

    override fun init(mat: Mat?) {
        a = AprilTagDetectorJNI.createApriltagDetector(Tag.tagFamily, )

        Imgproc.cvtColor(mat, greyscale, Imgproc.COLOR_RGBA2GRAY)
        super.init(mat)
    }

    override fun processFrame(input: Mat?): Mat {
        Imgproc.cvtColor(input, greyscale, Imgproc.COLOR_RGBA2GRAY)

    }


}