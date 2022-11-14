package org.firstinspires.ftc.teamcode.pipeline

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.imgproc.Imgproc
import org.opencv.core.Core as Cv
import org.opencv.core.Scalar
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.round

class PolePipeline(private val telemetry: Telemetry) : OpenCvPipeline() {
    companion object {
        val LO_YELLOW = Scalar(10.0, 125.0, 150.0)
        val HI_YELLOW = Scalar(35.0, 255.0, 255.0)
        val GREEN = Scalar(0.0, 255.0, 0.0, 255.0)
        val RED = Scalar(255.0, 0.0, 0.0)
    }

    override fun processFrame(input: Mat): Mat {
        val hsvMat = Mat().also { Imgproc.cvtColor(input, it, Imgproc.COLOR_RGB2HSV) }
        val mask = Mat().also { Cv.inRange(hsvMat, LO_YELLOW, HI_YELLOW, it) }
        val (contours, hierarchy) = Pair(mutableListOf<MatOfPoint>(), Mat()).also { (a, b) ->
            Imgproc.findContours(
                mask, a, b, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE
            )
        }

        for (contour in contours) {
            val m = Imgproc.moments(contour)
            if (m.m00 != 0.0) {
                val center = Point(m.m10 / m.m00, m.m01 / m.m00)
                Imgproc.circle(input, center, 1, RED, 3)
                String.format("(%5.2f, %5.2f)", center.x, center.y).let {
                    telemetry.addLine("$it : ${contour.width()}, ${contour.height()}")
                    Imgproc.putText(
                        input, it,
                        Point(
                            center.x - 20, center.y - 20
                        ),
                        Imgproc.FONT_HERSHEY_COMPLEX,
                        0.2,
                        Scalar(0.0, 0.0, 0.0),
                        1
                    )
                }
            }
//            val p2f = MatOfPoint2f().apply { contour.convertTo(this, CvType.CV_32S) }
//            val approx = MatOfPoint2f().also { Imgproc.approxPolyDP(p2f, it, p2f.width() * 0.01, true) }
        }


        Imgproc.drawContours(input, contours, -1, GREEN, 1)
        telemetry.update()

        return input
    }
}
