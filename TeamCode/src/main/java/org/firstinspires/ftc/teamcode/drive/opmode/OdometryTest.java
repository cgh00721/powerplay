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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ConeDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(group="drive")
public class OdometryTest extends LinearOpMode {

    ConeDetectionPipeline detectionPipeline;
    int inch = 45;
    int col = 0;
    private  DcMotor lift = null;
     @Override
    public void runOpMode() throws InterruptedException {
        lift = hardwareMap.dcMotor.get("Lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        detectionPipeline = new ConeDetectionPipeline();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class,"isaac");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        camera.openCameraDevice();
        camera.startStreaming(320, 240);
        camera.setPipeline(detectionPipeline);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Trajectory ToCone = drive.trajectoryBuilder(new Pose2d(36.60, -66.28, Math.toRadians(90.00)))
                .splineTo(new Vector2d(36.6,-49.14), Math.toRadians(90.00))
                .build();
        Trajectory ToPole = drive.trajectoryBuilder(new Pose2d(36.6, -49.14, Math.toRadians(90.00)))
                .splineTo(new Vector2d(36.6,-66), Math.toRadians(90.00))
                .splineTo(new Vector2d(13.18, -61.49), Math.toRadians(180.47))
                .splineTo(new Vector2d(11.15, -36.23), Math.toRadians(94.59))
                .splineTo(new Vector2d(0.28, -34.39), Math.toRadians(88.22))
                .build();
        Trajectory ToPickup = drive.trajectoryBuilder(new Pose2d(0.28, -34.39, Math.toRadians(88.22)))
                .splineTo(new Vector2d(11.15, -13.18), Math.toRadians(88.06))
                .splineTo(new Vector2d(60.20, -11.15), Math.toRadians(2.37))
                .build();
        Trajectory BackToPole =  drive.trajectoryBuilder(new Pose2d(60.20, -11.15, Math.toRadians(2.37)))
                .splineTo(new Vector2d(11.15, -13.18), Math.toRadians(88.06))
                .splineTo(new Vector2d(0.28, -34.39), Math.toRadians(88.22))
                .build();
        Trajectory to1 = drive.trajectoryBuilder(new Pose2d(0.28, -34.39, Math.toRadians(88.22)))
                .splineTo(new Vector2d(11.89, -35.68), Math.toRadians(-0.88))
                .build();
        Trajectory to2 = drive.trajectoryBuilder(new Pose2d(0.28, -34.39, Math.toRadians(88.22)))
                .splineTo(new Vector2d(36.23, -35.86), Math.toRadians(-0.43))
                .build();
        Trajectory to3 = drive.trajectoryBuilder(new Pose2d(0.28, -34.39, Math.toRadians(88.22)))
                .splineTo(new Vector2d(61.12, -35.68), Math.toRadians(0.42))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(ToCone);

         if (detectionPipeline.getLatestResult() == 1) {
            col = 1;
        } else if (detectionPipeline.getLatestResult() == 2) {
            col = 2;
        } else if (detectionPipeline.getLatestResult() == 3) {
            col = 3;
        }

        drive.followTrajectory(ToPole);

        drive.followTrajectory(ToPickup);
        drive.followTrajectory(BackToPole);
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
}
