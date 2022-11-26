package org.firstinspires.ftc.teamcode.pipeline

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import org.opencv.core.Core as Cv


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


        val (contours, hierarchy) = Pair(mutableListOf<MatOfPoint>(), Mat()).also { (c, h) ->
            Imgproc.findContours(mask, c, h, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
        }
        Imgproc.drawContours(input, contours, -1, GREEN, 1)
        drawContourCenters(input, contours)

        val bounds =
            contours.map { Imgproc.minAreaRect(MatOfPoint2f.fromNativeAddr(it.nativeObjAddr)) }

        val widest = bounds.maxBy { it.boundingRect().width }

        Imgproc.drawContours(input, contours, bounds.indexOf(widest), RED, 2)

        telemetry.update()

        contours.forEach { it.release() }

        return input
    }

    private fun drawContourCenters(input: Mat, contours: List<MatOfPoint>) =
        contours.forEach { contour ->
            Imgproc.moments(contour).takeUnless { it.m00 == 0.0 }?.let {
                Imgproc.circle(input, Point(it.m10 / it.m00, it.m01 / it.m00), 1, RED, 3)
            }
        }
}
