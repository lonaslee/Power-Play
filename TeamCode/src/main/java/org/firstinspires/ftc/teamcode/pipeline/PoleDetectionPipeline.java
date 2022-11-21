package org.firstinspires.ftc.teamcode.pipeline;

import static org.firstinspires.ftc.teamcode.pipeline.PoleDetectionUtils.BLUE;
import static org.firstinspires.ftc.teamcode.pipeline.PoleDetectionUtils.GREEN;
import static org.firstinspires.ftc.teamcode.pipeline.PoleDetectionUtils.HI_YELLOW;
import static org.firstinspires.ftc.teamcode.pipeline.PoleDetectionUtils.LO_YELLOW;
import static org.firstinspires.ftc.teamcode.pipeline.PoleDetectionUtils.drawContourCenters;
import static org.firstinspires.ftc.teamcode.pipeline.PoleDetectionUtils.getApproximates;
import static org.firstinspires.ftc.teamcode.pipeline.PoleDetectionUtils.getRotatedRects;
import static org.firstinspires.ftc.teamcode.pipeline.PoleDetectionUtils.mat2fToMat;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public final class PoleDetectionPipeline extends OpenCvPipeline {
    private final Telemetry telemetry;

    public PoleDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Mat mask = new Mat();
        Core.inRange(hsv, LO_YELLOW, HI_YELLOW, mask);
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        drawContourCenters(input, contours);

        //        List<MatOfPoint> hulls = getConvexHulls(contours);

        int widestIdx = 0;
        List<RotatedRect> rects = getRotatedRects(contours);
        for (int i = 0; i < rects.size(); i++)
            if (rects.get(i).boundingRect().width > rects.get(widestIdx).boundingRect().width)
                widestIdx = i;

        ArrayList<MatOfPoint2f> approxs = getApproximates(contours);
        ArrayList<MatOfPoint> approxMats = new ArrayList<>();
        approxs.forEach(it -> approxMats.add(mat2fToMat(it)));
        Imgproc.drawContours(input, approxMats, -1, GREEN, 1);


//        int widestIdx = 0;
//        List<RotatedRect> approxRects = getRotatedRects(approxMats);
//        for (int i = 0; i < approxRects.size(); i++)
//            if (approxRects.get(i).boundingRect().width > approxRects.get(widestIdx).boundingRect().width)
//                widestIdx = i;


        telemetry.addData("as", approxs.size());

        Imgproc.drawContours(input, approxMats, widestIdx, BLUE, 2);

        contours.forEach(Mat::release);
        telemetry.update();

        return input;
    }
}
