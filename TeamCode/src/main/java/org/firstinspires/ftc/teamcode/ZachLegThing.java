package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Disabled
@Autonomous(name="ZachLegThing", group="Pushbot")
public class ZachLegThing extends LinearOpMode {
    private DcMotor motor0 = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor pump = null;

    private Servo servo = null;
    public void runOpMode() {
        motor0 = hardwareMap.dcMotor.get("motor0");
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        pump = hardwareMap.dcMotor.get("pump");
        servo = hardwareMap.servo.get("servo");

        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pump.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pump.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "waiting for start");
        telemetry.update();


        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        motor0.setTargetPosition(0);
        motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor0.setPower(1);
        motor1.setTargetPosition(5);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setPower(1);
        motor2.setTargetPosition(0);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setPower(1);


        sleep(300000000);

    }
}
