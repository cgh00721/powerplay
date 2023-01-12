package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**Encoder Ratio:
Aprox 0.3mm per pulse -- exact calculation: (37.5pi)/((((1+(46/17))) * (1+(46/17))) * 28)
scales to 1016 pulse per foot
**/
public class Encoder7959 extends AutonomousTest{
    private DcMotor LeftFrontDrive;
    private DcMotor LeftBackDrive;
    private DcMotor RightBackDrive;
    private DcMotor RightFrontDrive;

    public Encoder7959(DcMotor LFD, DcMotor RFD, DcMotor LBD, DcMotor RBD){
        this.LeftFrontDrive = LFD;
        this.RightBackDrive = RBD;
        this.RightFrontDrive = RFD;
        this.LeftBackDrive = LBD;

    }

    public void Forward(double power, int clicks){
        RightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBackDrive.setPower(power);
        RightFrontDrive.setPower(power);
        LeftBackDrive.setPower(power);
        LeftFrontDrive.setPower(power);
        RightBackDrive.setTargetPosition(clicks);
        LeftBackDrive.setTargetPosition(clicks);
        RightFrontDrive.setTargetPosition(clicks);
        LeftFrontDrive.setTargetPosition(clicks);
        while(LeftFrontDrive.isBusy() || RightBackDrive.isBusy() || RightFrontDrive.isBusy() || LeftBackDrive.isBusy()){}
        RightBackDrive.setPower(0);
        RightFrontDrive.setPower(0);
        LeftBackDrive.setPower(0);
        LeftFrontDrive.setPower(0);
    }
    public void TurnLeft(double power, int clicks){
        RightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBackDrive.setPower(power);
        RightFrontDrive.setPower(power);
        LeftBackDrive.setPower(-power);
        LeftFrontDrive.setPower(-power);
        RightBackDrive.setTargetPosition(clicks);
        LeftBackDrive.setTargetPosition(-clicks);
        RightFrontDrive.setTargetPosition(clicks);
        LeftFrontDrive.setTargetPosition(-clicks);
        while(LeftFrontDrive.isBusy() || RightBackDrive.isBusy() || RightFrontDrive.isBusy() || LeftBackDrive.isBusy()){}
        RightBackDrive.setPower(0);
        RightFrontDrive.setPower(0);
        LeftBackDrive.setPower(0);
        LeftFrontDrive.setPower(0);
    }
    public void TurnRight(double power, int clicks){
        RightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBackDrive.setPower(-power);
        RightFrontDrive.setPower(-power);
        LeftBackDrive.setPower(power);
        LeftFrontDrive.setPower(power);
        RightBackDrive.setTargetPosition(-clicks);
        LeftBackDrive.setTargetPosition(clicks);
        RightFrontDrive.setTargetPosition(-clicks);
        LeftFrontDrive.setTargetPosition(clicks);
        while(LeftFrontDrive.isBusy() || RightBackDrive.isBusy() || RightFrontDrive.isBusy() || LeftBackDrive.isBusy()){}
        RightBackDrive.setPower(0);
        RightFrontDrive.setPower(0);
        LeftBackDrive.setPower(0);
        LeftFrontDrive.setPower(0);
    }
    public void Reverse(double power, int clicks){
        RightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBackDrive.setPower(-power);
        RightFrontDrive.setPower(-power);
        LeftBackDrive.setPower(-power);
        LeftFrontDrive.setPower(-power);
        RightBackDrive.setTargetPosition(-clicks);
        LeftBackDrive.setTargetPosition(-clicks);
        RightFrontDrive.setTargetPosition(-clicks);
        LeftFrontDrive.setTargetPosition(-clicks);
        while(LeftFrontDrive.isBusy() || RightBackDrive.isBusy() || RightFrontDrive.isBusy() || LeftBackDrive.isBusy()){}
        RightBackDrive.setPower(0);
        RightFrontDrive.setPower(0);
        LeftBackDrive.setPower(0);
        LeftFrontDrive.setPower(0);
    }
    public void StrafeRight(double power, int clicks){
        RightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBackDrive.setPower(power);
        RightFrontDrive.setPower(-power);
        LeftBackDrive.setPower(-power);
        LeftFrontDrive.setPower(power);
        RightBackDrive.setTargetPosition(clicks);
        LeftBackDrive.setTargetPosition(-clicks);
        RightFrontDrive.setTargetPosition(-clicks);
        LeftFrontDrive.setTargetPosition(clicks);
        while(LeftFrontDrive.isBusy() || RightBackDrive.isBusy() || RightFrontDrive.isBusy() || LeftBackDrive.isBusy()){}
        RightBackDrive.setPower(0);
        RightFrontDrive.setPower(0);
        LeftBackDrive.setPower(0);
        LeftFrontDrive.setPower(0);
    }
    public void StrafeLeft(double power, int clicks){
        RightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBackDrive.setPower(-power);
        RightFrontDrive.setPower(power);
        LeftBackDrive.setPower(power);
        LeftFrontDrive.setPower(-power);
        RightBackDrive.setTargetPosition(-clicks);
        LeftBackDrive.setTargetPosition(clicks);
        RightFrontDrive.setTargetPosition(clicks);
        LeftFrontDrive.setTargetPosition(-clicks);
        while(LeftFrontDrive.isBusy() || RightBackDrive.isBusy() || RightFrontDrive.isBusy() || LeftBackDrive.isBusy()){}
        RightBackDrive.setPower(0);
        RightFrontDrive.setPower(0);
        LeftBackDrive.setPower(0);
        LeftFrontDrive.setPower(0);
    }
}
