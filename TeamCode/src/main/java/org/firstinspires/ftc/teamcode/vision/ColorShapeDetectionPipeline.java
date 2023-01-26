package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

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

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import androidx.annotation.NonNull;


@Config
public final class ColorShapeDetectionPipeline extends SignalSleevePipeline {
    public enum Shape {CIRCLE, TRIANGLE, RECTANGLE, UNKNOWN}

    public enum Color {YELLOW, CYAN, MAGENTA, UNKNOWN}

    @Nullable
    private final Telemetry telemetry;

    public ColorShapeDetectionPipeline(@NotNull Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public ColorShapeDetectionPipeline() {
        this.telemetry = null;
    }

    private final Queue<Shape> shapeHistory = new LinkedList<>();
    private final Queue<Color> colorHistory = new LinkedList<>();

    public Shape getShapeVerdict() {
        int circle = 0;
        int rectangle = 0;
        int triangle = 0;
        for (Shape shape : shapeHistory) {
            if (shape == Shape.CIRCLE) circle++;
            else if (shape == Shape.RECTANGLE) rectangle++;
            else if (shape == Shape.TRIANGLE) triangle++;
        }
        if (circle > rectangle && circle > triangle) return Shape.CIRCLE;
        if (rectangle > circle && rectangle > triangle) return Shape.RECTANGLE;
        return Shape.TRIANGLE;
    }

    public Color getColorVerdict() {
        int yellow = 0;
        int magenta = 0;
        int cyan = 0;
        for (Color color : colorHistory) {
            if (color == Color.YELLOW) yellow++;
            else if (color == Color.MAGENTA) magenta++;
            else if (color == Color.CYAN) cyan++;
        }
        if (yellow > magenta && yellow > cyan) return Color.YELLOW;
        if (magenta > yellow && magenta > cyan) return Color.MAGENTA;
        return Color.CYAN;
    }

    @NonNull
    @Override
    public Tag getVerdict() {
        Color colorVerdict = getColorVerdict();
        Shape shapeVerdict = getShapeVerdict();
        if (colorVerdict == Color.YELLOW && shapeVerdict == Shape.CIRCLE) return Tag.LEFT;
        if (colorVerdict == Color.MAGENTA && shapeVerdict == Shape.RECTANGLE) return Tag.MIDDLE;
        if (colorVerdict == Color.CYAN && shapeVerdict == Shape.TRIANGLE) return Tag.RIGHT;

        // something is goofy now
        if (shapeVerdict == Shape.UNKNOWN) return Tag.values()[colorVerdict.ordinal()];
        else return Tag.values()[shapeVerdict.ordinal()];
    }

    private Mat inputroi = new Mat();

    public static int topX = 400;
    public static int topY = 400;
    public static int height = 120;
    public static int width = 100;

    @NotNull
    @Override
    public Mat processFrame(Mat input) {
        reset();
        Rect roi = new Rect(topX, topY, width, height);
        Imgproc.rectangle(input, roi, DetectionUtils.GREEN);
        inputroi = new Mat(input, roi);

        Shape shape = findShape();
        Color color = findColors();

        if (telemetry != null) {
            telemetry.addData("shape", shape.name());
            telemetry.addData("color", color.name());
            telemetry.addData("shapeVerdict", getShapeVerdict().name());
            telemetry.addData("colorVerdict", getColorVerdict().name());
//            telemetry.update();
        }
        return input;
    }

    private static final Size blurSize = new Size(5, 7);

    private final Mat edges = new Mat();
    private final Mat grey = new Mat();
    private final Mat hierarchy = new Mat();
    private final List<MatOfPoint> contours = new ArrayList<>();

    @NotNull
    private Shape findShape() {
        Imgproc.cvtColor(inputroi, grey, Imgproc.COLOR_RGB2GRAY);
        Imgproc.GaussianBlur(grey, grey, blurSize, 0);

        Imgproc.Canny(grey, edges, 50, 100);

        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        DetectionUtils.filterByArea(contours, 200);

        List<MatOfPoint2f> approxs = DetectionUtils.getApproximates(contours, 0.04);

        Imgproc.drawContours(inputroi, DetectionUtils.mat2fToMatList(approxs), -1, DetectionUtils.GREEN);

        if (approxs.size() == 0) return Shape.UNKNOWN;
        MatOfPoint2f approx = Collections.min(approxs, Comparator.comparing(MatOfPoint2f::total));

        if (shapeHistory.size() >= 30) shapeHistory.poll();

        if (approx.total() == 3) {
            shapeHistory.add(Shape.TRIANGLE);
            return Shape.TRIANGLE;
        }
        if (approx.total() == 4) {
            shapeHistory.add(Shape.RECTANGLE);
            return Shape.RECTANGLE;
        }
        shapeHistory.add(Shape.CIRCLE);
        return Shape.CIRCLE;
    }

    @NotNull
    private Color findColor() {
        Scalar sumColors = Core.sumElems(inputroi);
        double minColor = Math.min(sumColors.val[0], Math.min(sumColors.val[1], sumColors.val[2]));

        if (sumColors.val[0] == minColor) return Color.CYAN;
        if (sumColors.val[1] == minColor) return Color.MAGENTA;
        if (sumColors.val[2] == minColor) return Color.YELLOW;
        return Color.UNKNOWN;
    }

    private static final Scalar[] CYAN = new Scalar[]{new Scalar(80, 100, 30), new Scalar(100, 255, 255)};
    private static final Scalar[] MAGENTA = new Scalar[]{new Scalar(140, 50, 50), new Scalar(180, 255, 255)};
    private static final Scalar[] YELLOW = new Scalar[]{new Scalar(20, 100, 100), new Scalar(30, 255, 255)};

    private final Mat hsv = new Mat();
    private final Mat kernel = new Mat();
    private final Mat cyan = new Mat();
    private final Mat magenta = new Mat();
    private final Mat yellow = new Mat();
    private final List<Mat> masks = List.of(cyan, magenta, yellow);

    private final List<MatOfPoint> cyanc = new ArrayList<>();
    private final List<MatOfPoint> magentac = new ArrayList<>();
    private final List<MatOfPoint> yellowc = new ArrayList<>();

    @NotNull
    private Color findColors() {
        Imgproc.cvtColor(inputroi, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, CYAN[0], CYAN[1], cyan);
        Core.inRange(hsv, MAGENTA[0], MAGENTA[1], magenta);
        Core.inRange(hsv, YELLOW[0], YELLOW[1], yellow);

        for (Mat mask : masks) {
            Imgproc.GaussianBlur(mask, mask, new Size(3, 3), 0);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);
        }

        cyanc.clear();
        magentac.clear();
        yellowc.clear();
        Imgproc.findContours(cyan, cyanc, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(magenta, magentac, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(yellow, yellowc, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        DetectionUtils.filterByArea(cyanc, 100);
        DetectionUtils.filterByArea(magentac, 100);
        DetectionUtils.filterByArea(yellowc, 100);

        double cn = 0;
        double mn = 0;
        double yn = 0;

        if (cyanc.size() != 0) {
            Imgproc.drawContours(inputroi, cyanc, -1, DetectionUtils.GREEN);
            cn = Imgproc.contourArea(Collections.max(cyanc, Comparator.comparing(Imgproc::contourArea)));
        }
        if (magentac.size() != 0) {
            Imgproc.drawContours(inputroi, magentac, -1, DetectionUtils.GREEN);
            mn = Imgproc.contourArea(Collections.max(magentac, Comparator.comparing(Imgproc::contourArea)));
        }
        if (yellowc.size() != 0) {
            Imgproc.drawContours(inputroi, yellowc, -1, DetectionUtils.GREEN);
            yn = Imgproc.contourArea(Collections.max(yellowc, Comparator.comparing(Imgproc::contourArea)));
        }
        if (telemetry != null) {
            telemetry.addData("cyan", cn);
            telemetry.addData("magenta", mn);
            telemetry.addData("yellow", yn);
        }

        if (cn + mn + yn == 0) return Color.UNKNOWN;
        if (colorHistory.size() >= 30) colorHistory.poll();

        if (cn > mn && cn > yn) {
            colorHistory.add(Color.CYAN);
            return Color.CYAN;
        }
        if (mn > yn && mn > cn) {
            colorHistory.add(Color.MAGENTA);
            return Color.MAGENTA;
        }
        colorHistory.add(Color.YELLOW);
        return Color.YELLOW;
    }

    private void reset() {
        grey.release();
        hierarchy.release();
        edges.release();

        contours.clear();
    }
}
