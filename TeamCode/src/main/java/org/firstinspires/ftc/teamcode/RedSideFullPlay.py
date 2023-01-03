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


@Autonomous(name="RedSideFullPlay", group="Pushbot")
public class RedSideFullPlay extends LinearOpMode {
    ConeDetectionPipeline detectionPipeline;
    private DcMotor RightFrontDrive = null;
    private DcMotor LeftFrontDrive = null;
    private DcMotor LeftBackDrive = null;
    private DcMotor RightBackDrive = null;
    int inch = 45;
    int col = 0;
    private  DcMotor lift = null;
    public void runOpMode() {
        RightFrontDrive = hardwareMap.dcMotor.get("RFD");
        LeftFrontDrive = hardwareMap.dcMotor.get("LFD");
        LeftBackDrive = hardwareMap.dcMotor.get("LBD");
        RightBackDrive = hardwareMap.dcMotor.get("RBD");
        lift = hardwareMap.dcMotor.get("Lift");

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
        lift.setPower(1);
        lift.setTargetPosition(700);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(600);
        robot.Forward(0.5 ,14*(inch));
        sleep(500);
// blockChain
        if (detectionPipeline.getLatestResult() == 1) {
            col = 1;
        } else if (detectionPipeline.getLatestResult() == 2) {
            col = 2;
        } else if (detectionPipeline.getLatestResult() == 3) {
            col = 3;
        }
    //navigate to high junction
    robot.Reverse(0.5,13*(inch));
    robot.StrafeLeft(0.5, 6*(inch));
    robot.Forward(0.5, 12*(inch));
    robot.StrafeLeft(0.5,2*(inch));
    //Claw moves
    robot.Reverse(0.5,1*(inch));
    robot.StrafeRight(0.5,2*(inch));
    robot.TurnLeft(0.5,13*(inch));
    robot.Forward(0.5,12*(inch));    
    //claw moves
    for(int i = 0; i < 3; i++){
        robot.Reverse(0.5,12*(inch));
        robot.TurnLeft(0.5,13*(inch));
        robot.StrafeLeft(0.5,2*(inch));
        robot.Forward(0.5,1*(inch));
        //Claw moves
        robot.Reverse(0.5,1*(inch));
        robot.StrafeRight(0.5,2*(inch));
        robot.TurnLeft(0.5,13*(inch));
        robot.Forward(0.5,12*(inch));    
        //claw moves    
    }
    if(col == 1){
        
    }
    else if(col ==2){
    
    }
    else{
        
    }
    
    }
}
