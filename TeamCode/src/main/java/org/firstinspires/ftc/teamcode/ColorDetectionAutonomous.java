package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;

        import org.openftc.easyopencv.OpenCvCamera;
        import org.openftc.easyopencv.OpenCvCameraFactory;
        import org.openftc.easyopencv.OpenCvInternalCamera;
        import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name="ColorDetectionAutonomous", group="Pushbot")
public class ColorDetectionAutonomous extends LinearOpMode {
    ConeDetectionPipeline detectionPipeline;
    private DcMotor RightFrontDrive = null;
    private DcMotor LeftFrontDrive = null;
    private DcMotor LeftBackDrive = null;
    private DcMotor RightBackDrive = null;
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
        RightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        LeftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        RightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        LeftBackDrive.setDirection(DcMotor.Direction.REVERSE);


        RightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "waiting for start");
        telemetry.update();

        detectionPipeline = new ConeDetectionPipeline();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //WebcamName webcamName = hardwareMap.get(WebcamName.class,"NAME_OF_CAMERA_IN_CONFIG_FILE");

        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName,cameraMonitorViewId);
        camera.openCameraDevice();
        camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);

        camera.setPipeline(detectionPipeline);

        //int detected_position = 0;

        //while ((detected_position = detectionPipeline.getLatestResult()) == 0) {}
        Encoder7959 robot = new Encoder7959(LeftFrontDrive,RightFrontDrive,LeftBackDrive,RightBackDrive);
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();
        sleep(500);
        robot.Reverse(0.5,1000);
        sleep(100);
        robot.Forward(0.5,1000);
        robot.StrafeRight(0.5,800);
        robot.Reverse(0.5,300);
        sleep(500);
        if (detectionPipeline.getLatestResult() == 1) {
            telemetry.addData("Result", detectionPipeline.getLatestResult());
            telemetry.update();
            robot.StrafeRight(0.5, 1100);
            robot.Forward(0.5,1200);
            sleep(5000);
        } else if (detectionPipeline.getLatestResult() == 2) {
            telemetry.addData("Result", detectionPipeline.getLatestResult());
            telemetry.update();
            robot.StrafeRight(0.5, 1000);
            sleep(5000);
        } else if (detectionPipeline.getLatestResult() == 3) {
            telemetry.addData("Result", detectionPipeline.getLatestResult());
            telemetry.update();
            robot.StrafeRight(0.5,1000);
            robot.Reverse(0.5,1200);
            sleep(5000);
        }
        }
}
