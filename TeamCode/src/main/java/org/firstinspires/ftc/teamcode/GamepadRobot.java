/*public class GamepadRobot {
    public DcMotor lFD;
    public DcMotor rFD;
    public DcMotor lBD;
    public DcMotor rBD;

    public GamepadRobot(HardwareMap hardwareMap) {
        lBD = hardwareMap.dcMotor.get("LBD");
        rBD = hardwareMap.dcMotor.get("RBD");
        lFD = hardwareMap.dcMotor.get("LFD");
        rFD = hardwareMap.dcMotor.get("RFD");
        
        lBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        lBD.setDirection(DcMotor.Direction.REVERSE);
        lFD.setDirection(DcMotor.Direction.REVERSE);
    }

    public void handleGamepad(Gamepad gamepad1) {
        handleGamepad(
            gamepad1.left_stick_x,
            gamepad1.left_stick_y,
            gamepad1.right_bumper,
            gamepad1.left_bumper,
            /*gamepad1.a,
            gamepad1.b,
            gamepad1.x,
            gamepad1.y/
        );
    }

    public void handleGamepad(
        float left_stick_x, 
        float left_stick_y, 
        boolean right_bumper, 
        boolean left_bumper,
        /*boolean a,
        boolean b,
        boolean x,
        boolean y/
    ) {
        if (right_bumper) {
            lFD.setPower(0.5);
            lBD.setPower(0.5);
            rFD.setPower(-0.5);
            rBD.setPower(-0.5);
            return;
        } else
        if (left_bumper) {
            lFD.setPower(-0.5);
            lBD.setPower(-0.5);
            rFD.setPower(0.5);
            rBD.setPower(0.5);
            return;
        }
        
        lBD.setPower(Math.pow(left_stick_y, 5) + Math.pow(left_stick_x, 5));// * 0.15);
        lFD.setPower(Math.pow(left_stick_y, 5) - Math.pow(left_stick_x, 5));// * 0.15);
        rBD.setPower(Math.pow(left_stick_y, 5) - Math.pow(left_stick_x, 5));// * 0.15);
        rFD.setPower(Math.pow(left_stick_y, 5) + Math.pow(left_stick_x, 5));// * 0.15);
    }
}*/

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class GamepadRobot {

    private DcMotor RightFrontDrive = null;
    private DcMotor LeftFrontDrive = null;
    private DcMotor LeftBackDrive = null;
    private DcMotor RightBackDrive = null;
    private DcMotor Lift = null;
    private Servo claw = null;
    private ColorSensor top;
    private ColorSensor bottom;

    boolean turbo;
    int encoderlocation;
    public GamepadRobot(HardwareMap hardwareMap)
    {
        //telemetry.addData("status", "Initialized");
        //telemetry.update();

        RightFrontDrive = hardwareMap.dcMotor.get("RFD");
        LeftFrontDrive = hardwareMap.dcMotor.get("LFD");
        LeftBackDrive = hardwareMap.dcMotor.get("LBD");
        RightBackDrive = hardwareMap.dcMotor.get("RBD");
        Lift = hardwareMap.dcMotor.get("Lift");
        claw = hardwareMap.servo.get("claw");
        bottom = hardwareMap.get(ColorSensor.class, "bottom");
        top = hardwareMap.get(ColorSensor.class, "top");
        RightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //telemetry.update();
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
        encoderlocation = -1;
        turbo = true;

    }

    /*public void handleGamepad(Gamepad gamepad1) {
        handleGamepad(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.left_bumper,
                gamepad1.right_bumper,
                gamepad1.a,
                gamepad1.b,
                gamepad1.x,
                gamepad1.y,
                gamepad1.left_trigger,
                gamepad1.right_trigger,
                gamepad1.dpad_down,
                gamepad1.dpad_up,
                gamepad1.dpad_left,
                gamepad1.dpad_right,

                0f, 0f,
                false, false,
                false, false, false, false,
                0f, 0f,
                false, false, false, false
        );
    }

    public void handleGamepad(Gamepad gamepad1, Gamepad gamepad2) {
        handleGamepad(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.left_bumper,
                gamepad1.right_bumper,
                gamepad1.a,
                gamepad1.b,
                gamepad1.x,
                gamepad1.y,
                gamepad1.left_trigger,
                gamepad1.right_trigger,
                gamepad1.dpad_down,
                gamepad1.dpad_up,
                gamepad1.dpad_left,
                gamepad1.dpad_right,

                gamepad2.left_stick_x,
                gamepad2.left_stick_y,
                gamepad2.left_bumper,
                gamepad2.right_bumper,
                gamepad2.a,
                gamepad2.b,
                gamepad2.x,
                gamepad2.y,
                gamepad2.left_trigger,
                gamepad2.right_trigger,
                gamepad2.dpad_down,
                gamepad2.dpad_up,
                gamepad2.dpad_left,
                gamepad2.dpad_right
        );
    }

    public void handleGamepad(
            float left_stick_x,
            float left_stick_y,
            boolean left_bumper,
            boolean right_bumper
    ) {
        handleGamepad(
                left_stick_x,
                left_stick_y,
                left_bumper,
                right_bumper,
                false, false, false, false,
                0f, 0f,
                false, false, false, false,

                0f, 0f,
                false, false,
                false, false, false, false,
                0f, 0f,
                false, false, false, false
        );
    }*/

    public void handleGamepad(
            /*float left_stick_x1,
            float left_stick_y1,
            boolean left_bumper1,
            boolean right_bumper1,
            boolean a1,
            boolean b1,
            boolean x1,
            boolean y1,
            float left_trigger1,
            float right_trigger1,
            boolean dpad_down1,
            boolean dpad_up1,
            boolean dpad_left1,
            boolean dpad_right1,

            float left_stick_x2,
            float left_stick_y2,
            boolean left_bumper2,
            boolean right_bumper2,
            boolean a2,
            boolean b2,
            boolean x2,
            boolean y2,
            float left_trigger2,
            float right_trigger2,
            boolean dpad_down2,
            boolean dpad_up2,
            boolean dpad_left2,
            boolean dpad_right2*/
            Gamepad gamepad1, Gamepad gamepad2
    ) {
        //telemetry.addData("Working", Lift.getCurrentPosition() +", " + encoderlocation +", " + bottom.blue());
        //telemetry.update();
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
        }  else if (!gamepad1.left_bumper && !gamepad1.right_bumper && gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0){
            LeftFrontDrive.setPower(0);
            LeftBackDrive.setPower(0);
            RightFrontDrive.setPower(0);
            RightBackDrive.setPower(0);
        }
        else if(!turbo){// && (old_x != left_stick_x1 || old_y != left_stick_y1)){
            old_x = gamepad1.left_stick_x;
            old_y = gamepad1.left_stick_y;
            LeftFrontDrive.setPower((Math.pow(gamepad1.left_stick_y, 5) - Math.pow(gamepad1.left_stick_x, 5)) * 0.15);
            LeftBackDrive.setPower((Math.pow(gamepad1.left_stick_y, 5) + Math.pow(gamepad1.left_stick_x, 5)) * 0.15);
            RightFrontDrive.setPower((Math.pow(gamepad1.left_stick_y, 5) + Math.pow(gamepad1.left_stick_x, 5)) * 0.15);
            RightBackDrive.setPower((Math.pow(gamepad1.left_stick_y, 5) - Math.pow(gamepad1.left_stick_x, 5)) * 0.15);
        }
        else if (turbo){ //&& (old_x != left_stick_x1 || old_y != left_stick_y1)){
            //Set Power using cubic control
            old_x = gamepad1.left_stick_x;
            old_y = gamepad1.left_stick_y;
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
        /*if(top.blue()<1500 &&( left_stick_y2>0.05)){
            Lift.setPower(left_stick_y2);
            //Lift.setPower(-0.5);
            encoderlocation = Lift.getCurrentPosition();
        }
        //&& bottom.blue()<1500
        else if(left_stick_y2<-0.05 ) {
            Lift.setPower(left_stick_y2);
            encoderlocation = Lift.getCurrentPosition();

        }

        else {*/
        /**if(encoderlocation != -1){
         Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         Lift.setTargetPosition(encoderlocation);
         Lift.setPower(0.5);
         }**/
        //Lift.setPower(0.0);
        //}

        /*if(top.blue()>1500){
            //may need to change num
            encoderlocation = 2000;
        }
        if(bottom.blue()>1500){
            encoderlocation = 0;
        }*/
        if(gamepad2.a){
            claw.setPosition(0.50);

        }
        if(gamepad2.b){
            claw.setPosition(0.0);
        }

//***************************************************************************
    }

    float old_x = 0f;
    float old_y = 0f;
}