package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;


@Autonomous(name="RedSideFullPlay", group="Pushbot")
public class RedSideFullPlay extends LinearOpMode {
    GimmickPipeline detectionPipeline;
    private DcMotor RightFrontDrive = null;
    private DcMotor LeftFrontDrive = null;
    private DcMotor LeftBackDrive = null;
    private DcMotor RightBackDrive = null;
    private Servo claw = null;

    int inch = 45;
    int col = 0;
    private  DcMotor lift = null;
    private ColorSensor bottom;
    public void runOpMode() {
        RightFrontDrive = hardwareMap.dcMotor.get("RFD");
        LeftFrontDrive = hardwareMap.dcMotor.get("LFD");
        LeftBackDrive = hardwareMap.dcMotor.get("LBD");
        RightBackDrive = hardwareMap.dcMotor.get("RBD");
        lift = hardwareMap.dcMotor.get("Lift");
        claw = hardwareMap.servo.get("claw");
        bottom = hardwareMap.get(ColorSensor.class, "bottom");

        RightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "waiting for start");
        telemetry.update();

        detectionPipeline = new GimmickPipeline();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class,"isaac");

        //OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        camera.openCameraDevice();
        camera.startStreaming(320, 240);

        camera.setPipeline(detectionPipeline);


        Encoder7959 robot = new Encoder7959(LeftFrontDrive,RightFrontDrive,LeftBackDrive,RightBackDrive);
        waitForStart();
        claw.setPosition(closed);
        telemetry.addData("Status", "Running");
        telemetry.update();
        sleep(300);
        lift.setTargetPosition(-700);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(-1);
        sleep(600);
        robot.Forward(0.5 ,12*(inch));
        sleep(500);
        robot.StrafeRight(0.3,2*(inch));
        sleep(300);
        if (detectionPipeline.latestResult == 1) {
            col = 1;
        } else if (detectionPipeline.latestResult == 2) {
            col = 2;
        } else if (detectionPipeline.latestResult == 3) {
            col = 3;
        }
        telemetry.addData("Status", col);
        telemetry.update();
        robot.StrafeLeft(0.3,2*(inch));

        //navigate to high junction
        /**
         * robot.Reverse(0.6,14*(inch));
         *     robot.StrafeLeft(0.7, 27*(inch));
         *     robot.Forward(0.7, 28*(inch));
         *     robot.StrafeLeft(0.7,17*(inch));
         *     robot.Forward(0.7,3*(inch));
         *     //Claw moves
         *     robot.Reverse(0.7,3*(inch));
         *     robot.StrafeRight(0.7,11*(inch));
         *     robot.Forward(0.7,24*(inch));
         *     robot.TurnLeft(0.7,19*(inch));
         *     robot.Forward(0.7,46*(inch));
         *     //claw moves
         *     robot.Reverse(0.7,43*(inch));
         *     robot.TurnLeft(0.7,19*(inch));
         *
         */
    robot.Forward(0.7,20*(inch));
    robot.Reverse(0.5,3*(inch));
    robot.StrafeLeft(0.7, 18*(inch));
    claw.setPosition(closed);
    lift.setTargetPosition(-1850);
    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    lift.setPower(-1);
    while(lift.isBusy()){}
    sleep(400);
    robot.Forward(0.7,(5*(inch)));
    sleep(400);
    claw.setPosition(open);
    sleep(400);
    robot.Reverse(0.5,(int)3.5*(inch));
    lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    robot.Reverse(0.7,4*(inch));
    claw.setPosition(closed);
    if(col == 1){
        telemetry.addData("Status", "1");
        telemetry.update();
        robot.StrafeLeft(0.7,13*(inch));
        robot.Forward(0.5,3*(inch));

    }
    else if(col == 2){
        telemetry.addData("Status", "2");
        telemetry.update();
        robot.StrafeRight(0.7,11*(inch));
        robot.Forward(0.5,3*(inch));
    }
    else{
        telemetry.addData("Status", "3");
        telemetry.update();
        robot.StrafeRight(0.7,41*(inch));
        robot.Forward(0.5,3*(inch));

    }
    
    }
    double open = 0.50;
    double closed = 0.0;
    private void grab(){
        claw.setPosition(open);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(0);
        lift.setPower(1);
        //might stall prgm b/c of constant readjustment
        while(lift.isBusy()){}
        claw.setPosition(closed);
    }
    private void drop(){
        claw.setPosition(closed);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(000);
        lift.setPower(-1);
        //might stall prgm b/c of constant readjustment
        while(lift.isBusy()){}
        claw.setPosition(open);
    }

}
