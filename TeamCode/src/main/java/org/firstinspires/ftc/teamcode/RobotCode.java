package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="RobotCode")

public class RobotCode extends LinearOpMode {

    private DcMotor RightFrontDrive = null;
    private DcMotor LeftFrontDrive = null;
    private DcMotor LeftBackDrive = null;
    private DcMotor RightBackDrive = null;
    private DcMotor Lift = null;
    private Servo claw = null;
    //private ColorSensor top;
    //private ColorSensor bottom;

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
        claw = hardwareMap.servo.get("claw");
        //bottom = hardwareMap.get(ColorSensor.class, "bottom");
        //top = hardwareMap.get(ColorSensor.class, "top");
        RightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int encoderlocation = -1;
        waitForStart();
        boolean turbo = true;
        while(opModeIsActive())
        {
            telemetry.addData("Working", Lift.getCurrentPosition() +", " + encoderlocation);
            telemetry.update();
//***************************************************************************
            if(gamepad1.right_bumper) {
                LeftFrontDrive.setPower(0.5);
                LeftBackDrive.setPower(0.5);
                RightFrontDrive.setPower(-0.5);
                RightBackDrive.setPower(-0.5);
            }
            else if(gamepad1.left_bumper) {
                LeftFrontDrive.setPower(-0.5);
                LeftBackDrive.setPower(-0.5);
                RightFrontDrive.setPower(0.5);
                RightBackDrive.setPower(0.5);
            }
            else if(!turbo){
                LeftFrontDrive.setPower((Math.pow(gamepad1.left_stick_y, 5) - Math.pow(gamepad1.left_stick_x, 5)) * 0.15);
                LeftBackDrive.setPower((Math.pow(gamepad1.left_stick_y, 5) + Math.pow(gamepad1.left_stick_x, 5)) * 0.15);
                RightFrontDrive.setPower((Math.pow(gamepad1.left_stick_y, 5) + Math.pow(gamepad1.left_stick_x, 5)) * 0.15);
                RightBackDrive.setPower((Math.pow(gamepad1.left_stick_y, 5) - Math.pow(gamepad1.left_stick_x, 5)) * 0.15);
            }
            else if (turbo){
                //Set Power using cubic control
                LeftFrontDrive.setPower((Math.pow(gamepad1.left_stick_y, 5) - Math.pow(gamepad1.left_stick_x, 5)) * 0.75);
                LeftBackDrive.setPower((Math.pow(gamepad1.left_stick_y, 5) + Math.pow(gamepad1.left_stick_x, 5)) * 0.75);
                RightFrontDrive.setPower((Math.pow(gamepad1.left_stick_y, 5) + Math.pow(gamepad1.left_stick_x, 5)) * 0.75);
                RightBackDrive.setPower((Math.pow(gamepad1.left_stick_y, 5) - Math.pow(gamepad1.left_stick_x, 5)) * 0.75);
            }
            if(gamepad1.a){
                if(!turbo){
                    turbo = true;
                }
                else {
                    turbo = false;
                }
            }
            //top.blue()<1500 &&
            if((gamepad2.left_stick_y>0.05)){
                Lift.setPower(gamepad2.left_stick_y);
                encoderlocation = Lift.getCurrentPosition();
            }
            //&& bottom.blue()<1500
            else if(gamepad2.left_stick_y<-0.05 ) {
                Lift.setPower(gamepad2.left_stick_y);
                encoderlocation = Lift.getCurrentPosition();

            }

            else {
                /**if(encoderlocation != -1){
                    Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Lift.setTargetPosition(encoderlocation);
                    Lift.setPower(0.5);
                }**/
                Lift.setPower(0.0);
            }
            /**
            if(top.blue()>1500){
                //may need to change num
                encoderlocation = 2000;
            }
            if(bottom.blue()>1500){
                encoderlocation = 0;
            }
             */
            if(gamepad2.a){
                claw.setPosition(0.42);

            }
            if(gamepad2.b){
                claw.setPosition(0.0);
            }

//***************************************************************************

        }
    }
}
