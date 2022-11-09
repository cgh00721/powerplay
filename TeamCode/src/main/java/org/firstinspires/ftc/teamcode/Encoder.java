package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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


public class Encoder extends AutonomousTest{
    private DcMotor LeftFrontDrive;
    private DcMotor LeftBackDrive;
    private DcMotor RightBackDrive;
    private DcMotor RightFrontDrive;

    public Encoder(DcMotor LFD, DcMotor RFD, DcMotor LBD, DcMotor RBD){
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
