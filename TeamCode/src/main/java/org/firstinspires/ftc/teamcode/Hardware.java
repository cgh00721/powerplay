package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware {
    public DcMotor RightFrontDrive = null;
    public DcMotor RightBackDrive  = null;
    public DcMotor LeftFrontDrive  = null;
    public DcMotor LeftBackDrive   = null;
    public ElapsedTime runtime = new ElapsedTime();
    HardwareMap hardwareMap = null;



    public Hardware(HardwareMap hwMap){
        initialize(hwMap);
    }


    private void initialize(HardwareMap hwMap){
        hardwareMap = hwMap;

        RightFrontDrive = hardwareMap.get(DcMotor.class,"RFD");
        RightBackDrive = hardwareMap.get(DcMotor.class,"RBD");
        LeftFrontDrive = hardwareMap.get(DcMotor.class,"LFD");
        LeftBackDrive = hardwareMap.get(DcMotor.class,"LBD");

        RightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        LeftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        RightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        LeftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        RightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RightFrontDrive.setPower(0);
        RightBackDrive.setPower(0);
        LeftBackDrive.setPower(0);
        LeftFrontDrive.setPower(0);
    }
}
