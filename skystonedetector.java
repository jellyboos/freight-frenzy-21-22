package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkystoneDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN
    }
    private Location location = Location.UNKNOWN;

    static final Rect LEFT_ROI = new Rect(
            new Point(0, 0),
            new Point(106, 239));
    static final Rect RIGHT_ROI = new Rect(
            new Point(107, 0),
            new Point(213, 239));
    static final Rect CENTER_ROI = new Rect(
            new Point(214, 0),
            new Point(319, 239));

    public SkystoneDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(65,100,100);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat center = mat.submat(CENTER_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0];
        double centerValue = Core.sumElems(center).val[0];
        double rightValue = Core.sumElems(right).val[0];

        left.release();
        center.release();
        right.release();

        telemetry.addData("Left raw value",leftValue);
        telemetry.addData("Center raw value", centerValue);
        telemetry.addData("Right raw value", rightValue);
        double maxValue = Math.max(Math.max(leftValue, centerValue), rightValue);

        if (maxValue == centerValue) {
            location = Location.CENTER;
        }
        else if (maxValue == rightValue) {
            location = Location.RIGHT;

            // a b c
            // max(a,b,c) X
            // max(a,b) = ??
            // max(??, c)

        }
        else if (maxValue == leftValue) {
            location = Location.LEFT;
        }
        telemetry.addData("Location", location.toString().toLowerCase());
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar red = new Scalar(255, 0, 0); // condition ? value1 : value2
        Scalar green = new Scalar(0, 255, 0);
        Imgproc.rectangle(input, LEFT_ROI, location == Location.LEFT ? green : red);
        Imgproc.rectangle(input, CENTER_ROI, location == Location.CENTER ? green : red);
        Imgproc.rectangle(input, RIGHT_ROI, location == Location.RIGHT ? green : red);

        return input;
    }

    public Location getLocation() {
        return location;
    }
}
