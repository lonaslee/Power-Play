package org.firstinspires.ftc.teamcode.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public final class ShapeDetectionPipeline extends OpenCvPipeline {
    public static Scalar LO_BOUND = new Scalar(50.0, 10.0, 0.0);
    public static Scalar HI_BOUND = new Scalar(200.0, 100.0, 255.0);
    public static Size blurSize = new Size(5, 7);

    @Nullable
    private final Telemetry telemetry;

    public ShapeDetectionPipeline() {
        this.telemetry = null;
    }

    public ShapeDetectionPipeline(@NotNull Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    private final Mat ycrcb = new Mat();
    private final Mat mask = new Mat();
    private final Mat hierarchy = new Mat();
    private final Mat kernel = new Mat();

    private final List<MatOfPoint> contours = new ArrayList<>();

    @NotNull
    @Override
    public Mat processFrame(@NotNull Mat input) {
        Rect roi = new Rect(input.width() * 5 / 12, input.height() * 5 / 12, input.width() / 6, input.height() / 6);
        Imgproc.rectangle(input, roi, DetectionUtils.GREEN);
        Mat inputroi = new Mat(input, roi);

        Imgproc.cvtColor(inputroi, ycrcb, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(ycrcb, LO_BOUND, HI_BOUND, mask);

        Imgproc.GaussianBlur(mask, mask, blurSize, 0);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        contours.clear();
        Imgproc.findContours(mask, contours, hierarchy, 0, 1);
        contours.removeIf(c -> Imgproc.contourArea(c) < 100);

        ArrayList<MatOfPoint2f> approxs = DetectionUtils.getApproximates(contours, 0.04);

        List<MatOfPoint> apxmint = approxs.stream().map(DetectionUtils::mat2fToMat)
                                          .collect(Collectors.toList());
        Imgproc.drawContours(inputroi, apxmint, -1, DetectionUtils.GREEN);

        if (telemetry != null) {
            for (MatOfPoint2f apx : approxs) {
                telemetry.addData("area", Imgproc.contourArea(apx));
                telemetry.addData("ns", apx.total());
                telemetry.addLine();
            }
            telemetry.update();
        }
        inputroi.release();
        return input;
    }
}
