package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="AutonomousTest", group="Pushbot")

public class AutonomousTest extends LinearOpMode{

    private DcMotor RightFrontDrive;
    private DcMotor LeftFrontDrive;
    private DcMotor LeftBackDrive;
    private DcMotor RightBackDrive;


    @Override
    public void runOpMode() {
        RightFrontDrive = hardwareMap.dcMotor.get("RFD");
        LeftFrontDrive = hardwareMap.dcMotor.get("LFD");
        LeftBackDrive = hardwareMap.dcMotor.get("LBD");
        RightBackDrive = hardwareMap.dcMotor.get("RBD");

        RightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        LeftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        RightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        LeftBackDrive.setDirection(DcMotor.Direction.FORWARD);


        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Encoder7959 robot = new Encoder7959(LeftFrontDrive,RightFrontDrive,LeftBackDrive,RightBackDrive);
        waitForStart();
        robot.Forward(0.1,70);


    }

}