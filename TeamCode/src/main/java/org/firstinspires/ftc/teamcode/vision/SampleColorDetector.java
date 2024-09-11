package org.firstinspires.ftc.teamcode.vision;

import androidx.annotation.NonNull;

import org.opencv.core.Core;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SampleColorDetector extends OpenCvPipeline {
    public enum SampleColor {
        RED,
        BLUE,
        YELLOW
    }

    private SampleColor color;

    public SampleColorDetector(@NonNull SampleColor color) {
        this.color = color;
    }

    public static int VIEW_DISPLAYED = 1;
    public static int ERODE_PASSES   = 9;

    public static volatile Scalar BOUNDING_RECTANGLE_COLOR = new Scalar(255, 0, 0);

    public static Scalar LOW_HSV_RANGE_BLUE  = new Scalar(97, 100, 0);
    public static Scalar HIGH_HSV_RANGE_BLUE = new Scalar(125, 255, 255);

    private static final Scalar LOW_HSV_RANGE_RED_ONE  = new Scalar(160, 100, 0);
    private static final Scalar HIGH_HSV_RANGE_RED_ONE = new Scalar(180, 255, 255);

    private static final Scalar LOW_HSV_RANGE_RED_TWO  = new Scalar(0, 100, 0);
    private static final Scalar HIGH_HSV_RANGE_RED_TWO = new Scalar(10, 255, 255);

    private static final Scalar LOW_HSV_RANGE_YELLOW = new Scalar(45, 100, 0);
    private static final Scalar HIGH_HSV_RANGE_YELLOW = new Scalar(55, 100, 255);

    private static final Point CV_ANCHOR        = new Point(-1, -1);
    private static final Scalar CV_BORDER_VALUE = new Scalar(-1);
    private static final int CV_BORDER_TYPE     = Core.BORDER_CONSTANT;

    private final Mat hsvMat          = new Mat(),
                      threshold0      = new Mat(),
                      threshold1      = new Mat(),
                      hierarchy       = new Mat(),
                      cvErodeKernel   = new Mat(),
                      thresholdOutput = new Mat(),
                      erodeOutput     = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        // Convert color to HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        switch (color) {
            case BLUE:
                Core.inRange(hsvMat, LOW_HSV_RANGE_BLUE, HIGH_HSV_RANGE_BLUE, thresholdOutput);
                break;
            case RED:
                Core.inRange(hsvMat, LOW_HSV_RANGE_RED_ONE, HIGH_HSV_RANGE_RED_ONE, threshold0);
                Core.inRange(hsvMat, LOW_HSV_RANGE_RED_TWO, HIGH_HSV_RANGE_RED_TWO, threshold0);
                Core.add(threshold0, threshold0, thresholdOutput);
                break;
            case YELLOW:
                Core.inRange(hsvMat, LOW_HSV_RANGE_YELLOW, HIGH_HSV_RANGE_YELLOW, thresholdOutput);
                break;
        }

        // Erode to remove noise
        Imgproc.erode(
                thresholdOutput,
                erodeOutput,
                cvErodeKernel,
                CV_ANCHOR,
                ERODE_PASSES,
                CV_BORDER_TYPE,
                CV_BORDER_VALUE
        );

        // Finds the contours of the image
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(
                erodeOutput,
                contours,
                hierarchy,
                Imgproc.RETR_TREE,
                Imgproc.CHAIN_APPROX_SIMPLE
        );

        // Creates bounding rectangles along all of the detected contours
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        // Draw a bounding box over all rectangles
        for (Rect rect : boundRect) {
           Imgproc.rectangle(hsvMat, rect, BOUNDING_RECTANGLE_COLOR);
        }

        if (VIEW_DISPLAYED == 1) {
            return input;
        } else if (VIEW_DISPLAYED == 2) {
            return threshold0;
        } else if (VIEW_DISPLAYED == 3) {
            return threshold1;
        } else if (VIEW_DISPLAYED == 4) {
            return thresholdOutput;
        } else if (VIEW_DISPLAYED == 5) {
            return erodeOutput;
        }

        return hsvMat;
    }

}