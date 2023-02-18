package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class GimmickPipeline extends OpenCvPipeline {
    public AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    public int latestResult;

    public GimmickPipeline() {
        super();
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat red = new Mat();
        Mat green = new Mat();
        Mat blue = new Mat();

        Core.extractChannel(input, red, 0);
        Core.extractChannel(input, green, 1);
        Core.extractChannel(input, blue, 2);

        Core.subtract(red, blue, red);
        Core.subtract(green, blue, green);

        Imgproc.cvtColor(red, red, Imgproc.COLOR_GRAY2RGBA);
        Imgproc.cvtColor(green, green, Imgproc.COLOR_GRAY2RGBA);

        aprilTagDetectionPipeline.processFrame(red);
        aprilTagDetectionPipeline.processFrame(green);

        latestResult = aprilTagDetectionPipeline.latestResult;

        return input;
    }
}
