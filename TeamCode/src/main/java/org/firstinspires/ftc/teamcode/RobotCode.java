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
            if(gamepad1.left_trigger>0) {
                LeftFrontDrive.setPower(1/2);
                LeftBackDrive.setPower(1/2);
                RightFrontDrive.setPower(-1/2);
                RightBackDrive.setPower(-1/2);
            }
            else if(gamepad1.right_trigger>0) {
                LeftFrontDrive.setPower(-1/2);
                LeftBackDrive.setPower(-1/2);
                RightFrontDrive.setPower(1/2);
                RightBackDrive.setPower(1/2);
            }
            else {
                //Set Power using cubic control
                LeftFrontDrive.setPower((Math.pow(gamepad1.left_stick_y,5) - Math.pow(gamepad1.left_stick_x,5))*0.75);
                LeftBackDrive.setPower((Math.pow(gamepad1.left_stick_y,5) + Math.pow(gamepad1.left_stick_x,5))*0.75);
                RightFrontDrive.setPower((Math.pow(gamepad1.left_stick_y,5) + Math.pow(gamepad1.left_stick_x,5))*0.75);
                RightBackDrive.setPower((Math.pow(gamepad1.left_stick_y,5) - Math.pow(gamepad1.left_stick_x,5))*0.75);
            }
//***************************************************************************

        }
    }
}
