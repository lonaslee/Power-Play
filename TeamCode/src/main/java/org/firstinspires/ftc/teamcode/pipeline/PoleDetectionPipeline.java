package org.firstinspires.ftc.teamcode.pipeline;

import static org.firstinspires.ftc.teamcode.pipeline.PoleDetectionUtils.GREEN;
import static org.firstinspires.ftc.teamcode.pipeline.PoleDetectionUtils.HI_YELLOW;
import static org.firstinspires.ftc.teamcode.pipeline.PoleDetectionUtils.LO_YELLOW;
import static org.firstinspires.ftc.teamcode.pipeline.PoleDetectionUtils.getApproximates;
import static org.firstinspires.ftc.teamcode.pipeline.PoleDetectionUtils.getRotatedRects;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public final class PoleDetectionPipeline extends OpenCvPipeline {
    private final Telemetry telemetry;

    public PoleDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        // get yellow mask
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Mat mask = new Mat();
        Core.inRange(hsv, LO_YELLOW, HI_YELLOW, mask);

        // get mat with only mask color range
        Mat bitAnd = new Mat();
        Core.bitwise_and(input, input, bitAnd, mask);

        // find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // approximate contours
        List<MatOfPoint2f> approx2f = getApproximates(contours);
        List<MatOfPoint> approxContours = approx2f.stream().map(PoleDetectionUtils::mat2fToMat).collect(Collectors.toList());
        List<RotatedRect> approxRects = getRotatedRects(approx2f);

        // filter out bad contours
        List<Integer> badIndices = new ArrayList<>();
        if (false) for (int i = 0; i < approxContours.size(); i++) {
            Point[] points = new Point[4]; // top left is origin, going positive down and right
            approxRects.get(i).points(points);

            double topHorizontal = points[1].x - points[0].x;
            double lowHorizontal = points[2].x - points[3].x;
            double rightVertical = points[2].y - points[1].y;
            double leftVertical = points[3].y - points[0].y;


            if (Stream.of(rightVertical, leftVertical).anyMatch(it -> it < 5)) {
                badIndices.add(i);
                telemetry.addData("TH", topHorizontal);
                telemetry.addData("LH", lowHorizontal);
                telemetry.addData("RV", rightVertical);
                telemetry.addData("LV", leftVertical);
            }
        }
        if (0 != 0) for (Integer badIndex : badIndices) {
            List.of(contours, approxContours, approxRects).forEach(it -> it.remove(badIndex));
        }

        telemetry.addLine(badIndices.toString());

        // add what this sees to the output image
//        drawContourCenters(input, contours);
        Imgproc.drawContours(input, approxContours, -1, GREEN, 1);


        // find widest contour, which should be the closest junction
        int widestIdx = 0;
        for (int i = 0; i < approxRects.size(); i++) {
            if (approxRects.get(i).boundingRect().width > approxRects.get(widestIdx).boundingRect().width)
                widestIdx = i;
        }

        for (RotatedRect rect : approxRects) {
            telemetry.addData(String.format("area %f - %f", rect.size.width, rect.size.height), rect.size.width * rect.size.height);
        }

        Point[] points = new Point[4];
        approxRects.get(widestIdx).points(points);

        telemetry.addData("wf", points[1].x - points[0].x);
        telemetry.addData("hf", points[3].y - points[0].y);

        telemetry.update();

        return input;
    }
}
