package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BarcodeDetectionPipeline;
import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.*;//package are annoying

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ConeDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config

@Autonomous(group="drive")
public class RedCorner extends LinearOpMode {
    // I did not calibrate it. Measurements will be wrong probably. Numbers are default for apriltag library

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

    GimmickPipeline detectionPipeline;
    int inch = 45;
    int col = 0;
    private  DcMotor lift = null;
    private Servo claw = null;
    @Override
    public void runOpMode() throws InterruptedException {
        GlobalTelemetry.telemetry = telemetry;

        lift = hardwareMap.dcMotor.get("Lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw = hardwareMap.servo.get("claw");

        detectionPipeline = new GimmickPipeline();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class,"isaac");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        camera.openCameraDevice();
        camera.startStreaming(320, 240);
        camera.setPipeline(detectionPipeline);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // TODO: update offset from wall in the y coord
        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(90.00));
        drive.setPoseEstimate(startPose);


        /**THESE COMMANDS ARE FOR USING THE COLOR DETECTION PIPELINE, AND ARE NOT CURRENTLY IMPLEMENTED
         Trajectory ToCone = drive.trajectoryBuilder(startPose)
         .lineTo(new Vector2d(36,-45),
         SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
         .build();
         //these strafes might need to change to actual strafes


         Trajectory ToPole = drive.trajectoryBuilder(ToCone.end())
         .splineToConstantHeading(new Vector2d(36,-30), Math.toRadians(90.00),
         SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
         .splineToConstantHeading(new Vector2d(36,-45), Math.toRadians(90.00),
         SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
         .splineTo(new Vector2d(36, -36), Math.toRadians(90.00),
         SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
         .splineTo(new Vector2d(12, -36), Math.toRadians(90.00),
         SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
         .splineTo(new Vector2d(12, -34), Math.toRadians(90.00),
         SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
         .splineToConstantHeading(new Vector2d(-3,-30),Math.toRadians(90.00),
         SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
         .build();
         */
        //might need to change the conztant heading one, dependz
        Trajectory ToPole1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-12,-60), Math.toRadians(90.00),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-12,-36),Math.toRadians(90.00),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(-12, -34), Math.toRadians(90.00),
                        SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(0,-30),Math.toRadians(90.00),
                        SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        //might need to change  the -14 the previouz and nezt
        Trajectory ToPickup = drive.trajectoryBuilder(ToPole1.end())
                .splineToConstantHeading(new Vector2d(-14,-40),Math.toRadians(90.00),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-14, -24), Math.toRadians(90.00),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(-60, -12), Math.toRadians(00.00),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory BackToPole =  drive.trajectoryBuilder(ToPickup.end())
                .splineToConstantHeading(new Vector2d(-17, -5), Math.toRadians(00.00),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-20,-40),Math.toRadians(00.00),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-5, -35), Math.toRadians(00.00),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //could change these to strafes if the turning is messy, also, y is off by a factor of 2 from previous one. watch for issues there
        Trajectory to1 = drive.trajectoryBuilder((BackToPole.end()).plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .strafeLeft(10)
                .build();
        Trajectory to2 = drive.trajectoryBuilder((BackToPole.end()).plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .strafeLeft(34)
                .build();
        Trajectory to3 = drive.trajectoryBuilder((BackToPole.end()).plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .strafeLeft(58)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        for(int i = 0; i<5; i++) {
            if (detectionPipeline.latestResult == 1) {
                col = 1;
                break;
            } else if (detectionPipeline.latestResult == 2) {
                col = 2;
                break;
            } else if (detectionPipeline.latestResult == 3) {
                col = 3;
                break;
            }
            sleep(200);
        }
        telemetry.addData("num",col);
        telemetry.update();

        sleep(500);
        drive.followTrajectory(ToPole1);
        sleep(500);
        drive.followTrajectory(ToPickup);
        sleep(500);
        drive.followTrajectory(BackToPole);
        drive.turn(Math.toRadians(90));
        sleep(500);

        if(col == 1){
            drive.followTrajectory(to1);
        }
        else if(col == 2){
            drive.followTrajectory(to2);
        }
        else{
            drive.followTrajectory(to3);
        }

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());

        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;



    }
    double open = 0.55;
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
        lift.setTargetPosition(100);
        lift.setPower(1);
        //might stall prgm b/c of constant readjustment
        while(lift.isBusy()){}
        claw.setPosition(open);
    }

}

