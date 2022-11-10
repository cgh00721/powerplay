package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;

import org.openftc.easyopencv.OpenCvPipeline;


public class ConeDetectionPipeline extends OpenCvPipeline {
    // CYAN = 1
    // MAGENTA = 2
    // YELLOW = 3

    public static final double[] CYAN = new double[]{0, 255, 255};
    public static final double[] MAGENTA = new double[]{255, 0, 255};
    public static final double[] YELLOW = new double[]{255, 255, 0};

    public static int latestResult = 0;

    public static double euclidianDistance(double[] color1, double[] color2) {
        return Math.sqrt(
            Math.pow(color1[0] - color2[0], 2) +
            Math.pow(color1[1] - color2[1], 2) +
            Math.pow(color1[2] - color2[2], 2)
        );
    }

    @Override
    public Mat processFrame(Mat input) {
        double[] rgba = input.get(input.cols()/2, input.rows()/2);

        double cyan_dist = euclidianDistance(rgba, CYAN);
        double magenta_dist = euclidianDistance(rgba, MAGENTA);
        double yellow_dist = euclidianDistance(rgba, YELLOW);

        if ((cyan_dist < magenta_dist) && (cyan_dist < yellow_dist)) {
            latestResult = 1;
        } else if ((magenta_dist < cyan_dist) && (magenta_dist < yellow_dist)) {
            latestResult = 2;
        } else if ((yellow_dist < cyan_dist) && (yellow_dist < magenta_dist)) {
            latestResult = 3;
        }
        
        return input;
    }

    public static int getLatestResult() {
        return latestResult;
    }
}