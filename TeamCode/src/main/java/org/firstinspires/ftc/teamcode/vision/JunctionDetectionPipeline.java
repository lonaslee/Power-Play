package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.DetectionUtils.*;
import static org.firstinspires.ftc.teamcode.vision.DetectionUtils.HI_YELLOW;


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
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class JunctionDetectionPipeline extends OpenCvPipeline {
    private final Telemetry telemetry;
    public JunctionDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    private int inputRows;
    private int inputCols;

    private void setInputSize(Mat input) {
        inputRows = input.rows();
        inputCols = input.cols();
    }

    @Override
    public Mat processFrame(Mat input) {
        setInputSize(input);
        // get yellow mask
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Mat mask = new Mat();
        Core.inRange(hsv, LO_YELLOW, HI_YELLOW, mask);
//        Imgproc.GaussianBlur(mask, mask, new Size(5, 15), 0);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, new Mat());

        // get mat with only mask color range
        Mat bitAnd = new Mat();
        Core.bitwise_and(input, input, bitAnd, mask);

        // find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        List<MatOfPoint2f> approx2fs = getApproximates(contours);
        List<MatOfPoint> approxContours = approx2fs.stream().map(DetectionUtils::mat2fToMat).collect(Collectors.toList());
        List<RotatedRect> approxRects = getRotatedRects(approx2fs);

        for (RotatedRect rect : approxRects) {
            Point[] pts = new Point[4];
            rect.points(pts);
        }

        drawContourCenters(input, approxContours);

        filter(contours, approxContours, approxRects);
        int widestIdx = findWidest(contours, approxContours, approxRects);

        Imgproc.rectangle(input, approxRects.get(widestIdx).boundingRect(), BLUE);

        telemetry.addData("WIDEST", widestIdx);
        telemetry.addData("WIDTH", approxRects.get(widestIdx).size.width);

        telemetry.update();
        return input;
    }

    private int findWidest(List<MatOfPoint> contours, List<MatOfPoint> approxContours, List<RotatedRect> approxRects) {
        int widestIdx = 0;
        double widest = 0;
        for (int i = 0; i < approxContours.size(); i++) {
            if (approxRects.get(i).size.width > widest) {
                widestIdx = i;
                widest = approxRects.get(i).size.width;
            }
        }
        return widestIdx;
    }

    private void filter(List<MatOfPoint> contours, List<MatOfPoint> approxContours, List<RotatedRect> approxRects) {
        List<Integer> badIndices = new ArrayList<>();
        for (int i = 0; i < contours.size(); i++) {
            if (Imgproc.contourArea(approxContours.get(i)) < 100) badIndices.add(i);
//            else if (approxRects.get(i).size.height < approxRects.get(i).size.width * 5) badIndices.add(i);
        }

        Collections.reverse(badIndices);
        for (int idx : badIndices) {
            contours.remove(idx);
            approxContours.remove(idx);
            approxRects.remove(idx);
        }
    }
}
