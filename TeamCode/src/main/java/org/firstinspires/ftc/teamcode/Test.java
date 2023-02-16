package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp
public class Test extends LinearOpMode {
    // I did not calibrate it. Measurements will be wrong probably.

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    @Override
    public void runOpMode() {
        GlobalTelemetry.telemetry = telemetry;
        //BarcodeDetectionPipeline detectionPipeline = new BarcodeDetectionPipeline();
        AprilTagDetectionPipeline detectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class,"isaac");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        camera.openCameraDevice();
        camera.startStreaming(320, 240);
        camera.setPipeline(detectionPipeline);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Result", detectionPipeline.latestResult);
            telemetry.update();
        }
    }
}
