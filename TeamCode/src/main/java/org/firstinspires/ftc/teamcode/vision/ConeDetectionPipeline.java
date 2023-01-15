package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.TestOnly;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

public class ConeDetectionPipeline extends OpenCvPipeline {
    public static final Scalar[] RED = new Scalar[]{new Scalar(0, 100, 20), new Scalar(10, 255, 255)};
    public static final Scalar[] BLUE = new Scalar[]{new Scalar(110, 50, 50), new Scalar(130, 255, 255)};
    public static final int MAX_OFFSET = 3;

    @Nullable
    private final Telemetry telemetry;

    @Nonnull
    private final Scalar[] bounds;

    @TestOnly
    public ConeDetectionPipeline(@Nullable Telemetry telemetry) {
        this.telemetry = telemetry;
        bounds = RED;
    }

    public ConeDetectionPipeline(@Nonnull Scalar[] color) {
        this.telemetry = null;
        bounds = color;
    }

    public ConeDetectionPipeline(@Nonnull Scalar[] color, @Nonnull Telemetry telemetry) {
        this.telemetry = telemetry;
        bounds = color;
    }

    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat kernel = new Mat();
    private final Mat hierarchy = new Mat();

    private final List<MatOfPoint> contours = new ArrayList<>();

    public double error = 0.0;
    public boolean detected = false;

    @Override
    public Mat processFrame(Mat input) {
        Rect sub = new Rect(0, input.height() / 5, input.width(), input.height() / 2);
        Imgproc.rectangle(input, sub, DetectionUtils.GREEN);

        double lo_offset = input.width() / 2.0 - MAX_OFFSET;
        double hi_offset = input.width() / 2.0 + MAX_OFFSET;
        Imgproc.line(input, new Point(lo_offset, 0), new Point(input.width() / 2.0 - MAX_OFFSET, input.height()), DetectionUtils.GREEN);
        Imgproc.line(input, new Point(hi_offset, 0), new Point(input.width() / 2.0 + MAX_OFFSET, input.height()), DetectionUtils.GREEN);

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsv, bounds[0], bounds[1], mask);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        contours.clear();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint approxCone = approximateMaxHull();
        detected = approxCone != null;
        if (!detected) {
            error = 0;
        } else {
            Imgproc.drawContours(input, List.of(approxCone), -1, DetectionUtils.BLUE);

            Point center = DetectionUtils.getContourCenter(approxCone);
            if (center != null) {
                double middle = center.x;
                Imgproc.line(input, new Point(middle, 0), new Point(middle, input.height()), DetectionUtils.RED);

                error = (lo_offset < middle && middle < hi_offset) ? 0 : input.width() / 2.0 - middle;
            }
        }

        if (telemetry != null) {
            telemetry.addData("error", error);
            telemetry.update();
        }
        return input;
    }

    @Nullable
    private MatOfPoint approximateMaxHull() {
        List<MatOfPoint> mint = DetectionUtils.getConvexHulls(contours);
        if (mint.size() == 0) return null;

        MatOfPoint maxAreaContour = Collections.max(mint, Comparator.comparing(Imgproc::contourArea));
        if (Imgproc.contourArea(maxAreaContour) < 1500) return null;

        MatOfPoint2f matAs2f = DetectionUtils.matToMat2f(maxAreaContour);
        MatOfPoint2f approx = new MatOfPoint2f();
        Imgproc.approxPolyDP(matAs2f, approx, 0.05 * Imgproc.arcLength(matAs2f, true), true);
        return DetectionUtils.mat2fToMat(approx);
    }
}
