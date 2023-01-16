package org.firstinspires.ftc.teamcode.vision;

import org.jetbrains.annotations.NotNull;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ShapeThreshDetectionPipeline extends OpenCvPipeline {
    public static double thresh = 100.0;
    public static double maxval = 255.0;

    private final Mat grey = new Mat();
    private final Mat threshed = new Mat();

    private final List<MatOfPoint> contours = new ArrayList<>();

    @Override
    @NotNull
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
        Imgproc.threshold(grey, threshed, thresh, maxval, Imgproc.THRESH_BINARY_INV | Imgproc.THRESH_OTSU);

        contours.clear();
        Imgproc.findContours(threshed, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, contours, -1, DetectionUtils.GREEN);

        return threshed;
    }
}
