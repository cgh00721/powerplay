package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.lang.Math;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="BlueSide", group="Pushbot")
public class BlueSide extends LinearOpMode {
    ConeDetectionPipeline detectionPipeline;
    private DcMotor RightFrontDrive = null;
    private DcMotor LeftFrontDrive = null;
    private DcMotor LeftBackDrive = null;
    private DcMotor RightBackDrive = null;
    int inch = 85;
    public void runOpMode() {
        RightFrontDrive = hardwareMap.dcMotor.get("RFD");
        LeftFrontDrive = hardwareMap.dcMotor.get("LFD");
        LeftBackDrive = hardwareMap.dcMotor.get("LBD");
        RightBackDrive = hardwareMap.dcMotor.get("RBD");

        RightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.update();
        //Set wheel diection
        RightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        LeftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        RightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        LeftBackDrive.setDirection(DcMotor.Direction.FORWARD);


        RightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "waiting for start");
        telemetry.update();

        detectionPipeline = new ConeDetectionPipeline();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class,"isaac");

        //OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        camera.openCameraDevice();
        camera.startStreaming(320, 240);

        camera.setPipeline(detectionPipeline);


        Encoder robot = new Encoder(LeftFrontDrive,RightFrontDrive,LeftBackDrive,RightBackDrive);
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        robot.Forward(0.5 ,17*(inch));
        sleep(500);
        if (detectionPipeline.getLatestResult() == 1) {
            telemetry.addData("Result", detectionPipeline.getLatestResult());
            telemetry.update();
            robot.StrafeLeft(0.5, 23*inch);
            sleep(5000);
        } else if (detectionPipeline.getLatestResult() == 2) {
            telemetry.addData("Result", detectionPipeline.getLatestResult());
            telemetry.update();
            robot.Forward(0.1,150);
            sleep(5000);
        } else if (detectionPipeline.getLatestResult() == 3) {
            telemetry.addData("Result", detectionPipeline.getLatestResult());
            telemetry.update();
            robot.StrafeRight(0.5, (23 * inch));
            sleep(5000);

        }
        /**robot.Reverse(0.5,1000);
        sleep(100);
        robot.Forward(0.5,1000);
        robot.StrafeRight(0.5,800);
        robot.Reverse(0.5,300);
        sleep(500);
        if (detectionPipeline.getLatestResult() == 1) {
            telemetry.addData("Result", detectionPipeline.getLatestResult());
            telemetry.update();
            robot.StrafeRight(0.5, 1100);
            robot.Forward(0.5,1300);
            sleep(5000);
        } else if (detectionPipeline.getLatestResult() == 2) {
            telemetry.addData("Result", detectionPipeline.getLatestResult());
            telemetry.update();
            robot.StrafeRight(0.5, 1000);
            robot.Forward(0.1,150);
            sleep(5000);
        } else if (detectionPipeline.getLatestResult() == 3) {
            telemetry.addData("Result", detectionPipeline.getLatestResult());
            telemetry.update();
            robot.StrafeRight(0.5,1000);
            robot.Reverse(0.5,1050);
            sleep(5000);
        }
        **/
    }
}
