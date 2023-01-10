package org.firstinspires.ftc.teamcode;

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

import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="OdometryTest", group="Pushbot")
public class OdometryTest extends LinearOpMode {
    ConeDetectionPipeline detectionPipeline;
    int inch = 45;
    int col = 0;
    private  DcMotor lift = null;
    @Override
    public void runOpMode() throws InterruptedException {
        /**
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
         **/
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(35.31, -60.75, Math.toRadians(90.00)))
                .splineTo(new Vector2d(36.23, -37.89), Math.toRadians(87.69))
                .splineTo(new Vector2d(64.26, -35.68), Math.toRadians(4.51))
                .splineTo(new Vector2d(59.83, -12.81), Math.toRadians(100.95))
                .splineTo(new Vector2d(35.12, -11.89), Math.toRadians(177.86))
                .splineTo(new Vector2d(12.63, -12.26), Math.toRadians(180.94))
                .splineTo(new Vector2d(11.34, -34.57), Math.toRadians(266.69))
                .splineTo(new Vector2d(9.86, -58.91), Math.toRadians(266.53))
                .splineTo(new Vector2d(35.49, -60.94), Math.toRadians(-2.46))
                .splineTo(new Vector2d(33.28, -59.83), Math.toRadians(86.82))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;



    }
}
