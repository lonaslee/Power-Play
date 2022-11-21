package org.firstinspires.ftc.teamcode.pipeline;

import org.jetbrains.annotations.NotNull;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class PoleDetectionUtils {
    public static final Scalar LO_YELLOW = new Scalar(10.0, 125.0, 150.0);
    public static final Scalar HI_YELLOW = new Scalar(35.0, 255.0, 255.0);
    public static final Scalar GREEN = new Scalar(0.0, 255.0, 0.0, 255.0);
    public static final Scalar RED = new Scalar(255.0, 0.0, 0.0);
    public static final Scalar BLUE = new Scalar(0.0, 0.0, 255.0);

    @NotNull
    public static List<MatOfPoint> getConvexHulls(@NotNull List<MatOfPoint> contours) {
        List<MatOfPoint> hulls = new ArrayList<>(contours.size());
        for (MatOfPoint contour : contours) {
            MatOfInt hullmat = new MatOfInt();
            Imgproc.convexHull(contour, hullmat);

            MatOfPoint hull = new MatOfPoint();
            hull.create(hullmat.rows(), 1, CvType.CV_32SC2);

            for (int i = 0; i < hullmat.rows(); i++)
                hull.put(i, 0, contour.get((int) hullmat.get(i, 0)[0], 0));
            hulls.add(hull);
        }
        return hulls;
    }

    @NotNull
    public static List<MatOfPoint> getContourBoundingRects(@NotNull List<RotatedRect> rects) {
        List<MatOfPoint> boxes = new ArrayList<>(rects.size());
        for (RotatedRect rect : rects) {
            Point[] verticies = new Point[4];
            rect.points(verticies);
            boxes.add(new MatOfPoint(verticies));
        }
        return boxes;
    }

    @NotNull
    public static List<RotatedRect> getRotatedRects(List<MatOfPoint> contours) {
        ArrayList<RotatedRect> rects = new ArrayList<>(contours.size());
        for (MatOfPoint contour : contours)
            rects.add(Imgproc.minAreaRect(matToMat2f(contour)));
        return rects;
    }


    @NotNull
    public static ArrayList<MatOfPoint2f> getApproximates(List<MatOfPoint> contours) {
        ArrayList<MatOfPoint2f> approxs = new ArrayList<>(contours.size());
        for (MatOfPoint contour : contours) {
            MatOfPoint2f mat2f = matToMat2f(contour);
            double epilison = 0.2 * Imgproc.arcLength(mat2f, true);
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(mat2f, approx, epilison, true);
            approxs.add(approx);
        }
        return approxs;
    }

    @NotNull
    public static MatOfPoint2f matToMat2f(@NotNull MatOfPoint matOfPoint) {
        MatOfPoint2f matOfPoint2f = new MatOfPoint2f();
        matOfPoint2f.fromArray(matOfPoint.toArray());
        return matOfPoint2f;
    }

    @NotNull
    public static MatOfPoint mat2fToMat(@NotNull MatOfPoint2f matOfPoint2f) {
        MatOfPoint matOfPoint = new MatOfPoint();
        matOfPoint.fromArray(matOfPoint2f.toArray());
        return matOfPoint;
    }

    public static void drawContourCenters(@NotNull Mat input, @NotNull List<MatOfPoint> contours) {
        for (MatOfPoint contour : contours) {
            Moments moment = Imgproc.moments(contour);
            if (moment.m00 != 0.0)
                Imgproc.circle(input, new Point(moment.m10 / moment.m00, moment.m01 / moment.m00), 1, RED, 3);
        }
    }
}
