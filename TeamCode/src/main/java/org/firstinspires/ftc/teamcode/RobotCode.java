package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="RobotCode")

public class RobotCode extends LinearOpMode {

    private DcMotor RightFrontDrive = null;
    private DcMotor LeftFrontDrive = null;
    private DcMotor LeftBackDrive = null;
    private DcMotor RightBackDrive = null;
    private DcMotor Lift = null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("status", "Initialized");
        telemetry.update();

        RightFrontDrive = hardwareMap.dcMotor.get("RFD");
        LeftFrontDrive = hardwareMap.dcMotor.get("LFD");
        LeftBackDrive = hardwareMap.dcMotor.get("LBD");
        RightBackDrive = hardwareMap.dcMotor.get("RBD");
        Lift = hardwareMap.dcMotor.get("Lift");
        RightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        waitForStart();

        while(opModeIsActive())
        {
            telemetry.addData("Working", "Working");
            telemetry.update();
//***************************************************************************
            if(gamepad1.left_bumper) {
                LeftFrontDrive.setPower(0.5);
                LeftBackDrive.setPower(0.5);
                RightFrontDrive.setPower(-0.5);
                RightBackDrive.setPower(-0.5);
            }
            else if(gamepad1.right_bumper) {
                LeftFrontDrive.setPower(-0.5);
                LeftBackDrive.setPower(-0.5);
                RightFrontDrive.setPower(0.5);
                RightBackDrive.setPower(0.5);
            }
            else {
                //Set Power using cubic control
                LeftFrontDrive.setPower((Math.pow(gamepad1.left_stick_y, 5) - Math.pow(gamepad1.left_stick_x, 5)) * 0.75);
                LeftBackDrive.setPower((Math.pow(gamepad1.left_stick_y, 5) + Math.pow(gamepad1.left_stick_x, 5)) * 0.75);
                RightFrontDrive.setPower((Math.pow(gamepad1.left_stick_y, 5) + Math.pow(gamepad1.left_stick_x, 5)) * 0.75);
                RightBackDrive.setPower((Math.pow(gamepad1.left_stick_y, 5) - Math.pow(gamepad1.left_stick_x, 5)) * 0.75);
            }

            if(gamepad1.dpad_up && Lift.getCurrentPosition() <= 2000){
                Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Lift.setPower(0.2);
            }
            else if(gamepad1.dpad_down && Lift.getCurrentPosition() >= 0) {
                Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Lift.setPower(-0.2);
            }
            else {
                Lift.setPower(0);
            }

//***************************************************************************

        }
    }
}
