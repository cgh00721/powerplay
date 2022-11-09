package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;


@TeleOp(name="RobotCode")

public class RobotCode extends LinearOpMode {

    private DcMotor RightFrontDrive = null;
    private DcMotor LeftFrontDrive = null;
    private DcMotor LeftBackDrive = null;
    private DcMotor RightBackDrive = null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("status", "Initialized");
        telemetry.update();

        RightFrontDrive = hardwareMap.dcMotor.get("RFD");
        LeftFrontDrive = hardwareMap.dcMotor.get("LFD");
        LeftBackDrive = hardwareMap.dcMotor.get("LBD");
        RightBackDrive = hardwareMap.dcMotor.get("RBD");

        RightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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




        waitForStart();

        while(opModeIsActive())
        {
            telemetry.addData("Working", "Working");
            telemetry.update();
//***************************************************************************
            if(gamepad1.left_bumper) {
                LeftFrontDrive.setPower(1/1.75);
                LeftBackDrive.setPower(1/1.75);
                RightFrontDrive.setPower(-1/1.75);
                RightBackDrive.setPower(-1/1.75);
            }
            else if(gamepad1.right_bumper) {
                LeftFrontDrive.setPower(-1/1.75);
                LeftBackDrive.setPower(-1/1.75);
                RightFrontDrive.setPower(1/1.75);
                RightBackDrive.setPower(1/1.75);
            }
            else {
                //Set Power using cubic control
                LeftFrontDrive.setPower((Math.pow(gamepad1.left_stick_y,3) - Math.pow(gamepad1.left_stick_x,3))/2);
                LeftBackDrive.setPower((Math.pow(gamepad1.left_stick_y,3) + Math.pow(gamepad1.left_stick_x,3))/2);
                RightFrontDrive.setPower((Math.pow(gamepad1.left_stick_y,3) + Math.pow(gamepad1.left_stick_x,3))/2);
                RightBackDrive.setPower((Math.pow(gamepad1.left_stick_y,3) - Math.pow(gamepad1.left_stick_x,3))/2);
            }
//***************************************************************************
            /**
             * Old Logarithmic control scheme for robot
             * else if(gamepad1.left_stick_y <=-0.1 || gamepad1.left_stick_x !=0) {
             LeftFrontDrive.setPower(Math.log((gamepad1.left_stick_y*gamepad1.left_stick_y)-0.01) - (Math.log(((gamepad1.left_stick_x)*(gamepad1.left_stick_x))-0.01)));
             LeftBackDrive.setPower(Math.log((gamepad1.left_stick_y*gamepad1.left_stick_y)-0.01) + (Math.log(((gamepad1.left_stick_x)*(gamepad1.left_stick_x))-0.01)));
             RightFrontDrive.setPower(Math.log((gamepad1.left_stick_y*gamepad1.left_stick_y)-0.01) +(Math.log(((gamepad1.left_stick_x)*(gamepad1.left_stick_x))-0.01)));
             RightBackDrive.setPower(Math.log((gamepad1.left_stick_y*gamepad1.left_stick_y)-0.01) - (Math.log((gamepad1.left_stick_x*gamepad1.left_stick_x)-0.01)));
             }
             else if(gamepad1.left_stick_y >=0.1 || gamepad1.left_stick_x !=0){
             LeftFrontDrive.setPower(Math.log(gamepad1.left_stick_y+1.1)- Math.log(gamepad1.left_stick_x+1.1));
             LeftBackDrive.setPower(Math.log(gamepad1.left_stick_y+1.1) + Math.log(gamepad1.left_stick_x+1.1));
             RightFrontDrive.setPower(Math.log(gamepad1.left_stick_y+1.1) + Math.log(gamepad1.left_stick_x+1.1));
             RightBackDrive.setPower(Math.log(gamepad1.left_stick_y+1.1)- Math.log(gamepad1.left_stick_x+1.1));
             }
             else{
             LeftFrontDrive.setPower(0 - 0);
             LeftBackDrive.setPower(0 + 0);
             RightFrontDrive.setPower(0 + 0);
             RightBackDrive.setPower(0 - 0);
             }

             * */

        }
    }
}
