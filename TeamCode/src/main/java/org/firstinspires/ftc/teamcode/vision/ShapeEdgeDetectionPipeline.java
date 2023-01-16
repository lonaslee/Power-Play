package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


public final class ShapeEdgeDetectionPipeline extends OpenCvPipeline {
    @Nullable
    private final Telemetry telemetry;

    public ShapeEdgeDetectionPipeline(@NotNull Telemetry telemetry) {this.telemetry = telemetry;}
    public ShapeEdgeDetectionPipeline() {this.telemetry = null;}

    private static final Size blurSize = new Size(5, 7);

    private final Mat grey = new Mat();
    private final Mat hierarchy = new Mat();
    private final Mat edges = new Mat();

    private final List<MatOfPoint> contours = new ArrayList<>();

    @NotNull
    @Override
    public Mat processFrame(Mat input) {
        reset();
        Rect roi = new Rect(input.width() * 5 / 12, input.height() * 5 / 12, input.width() / 6, input.height() / 6);
        Imgproc.rectangle(input, roi, DetectionUtils.GREEN);
        Mat inputroi = new Mat(input, roi);

        Imgproc.cvtColor(inputroi, grey, Imgproc.COLOR_RGB2GRAY);
        Imgproc.GaussianBlur(grey, grey, blurSize, 0);

        Imgproc.Canny(grey, edges, 50, 100);

        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        DetectionUtils.filterByArea(contours, 200);

        List<MatOfPoint2f> approxs = DetectionUtils.getApproximates(contours, 0.04);

        Imgproc.drawContours(inputroi, DetectionUtils.mat2fToMatList(approxs), -1, DetectionUtils.GREEN);

        if (telemetry != null) {
            for (MatOfPoint2f approx : approxs)
                telemetry.addData("ns", approx.total());
            telemetry.update();
        }
        return input;
    }

    private void reset() {
        grey.release();
        hierarchy.release();
        edges.release();

        contours.clear();
    }
}
