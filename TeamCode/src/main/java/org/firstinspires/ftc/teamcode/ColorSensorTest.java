package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareDevice;
@Disabled
@TeleOp

public class ColorSensorTest extends LinearOpMode {
    private DcMotor lBD;
    private DcMotor lFD;
    private DcMotor rBD;
    private DcMotor rFD;

    public void runOpMode() {
        LineFollower lineFollower = new LineFollower(
            hardwareMap.get(ColorSensor.class, "left_colorsensor"),
            hardwareMap.get(ColorSensor.class, "right_colorsensor"),
            AllianceColor.RED
        );
        
        lBD = hardwareMap.dcMotor.get("LBD");
        lFD = hardwareMap.dcMotor.get("LFD");
        rBD = hardwareMap.dcMotor.get("RBD");
        rFD = hardwareMap.dcMotor.get("RFD");
        
        waitForStart();
        
        while (opModeIsActive()) {
            double lPower = lineFollower.getLeftPower();
            double rPower = lineFollower.getRightPower();
            lBD.setPower(lPower);
            lFD.setPower(lPower);
            rBD.setPower(-rPower);
            rFD.setPower(-rPower);
            telemetry.addData("left", lineFollower.getLeftValue());
            telemetry.addData("right", lineFollower.getRightValue());
            telemetry.addData("diff", lineFollower.getDifference());
            telemetry.addData("lpower", lPower);
            telemetry.addData("rpower", rPower);
            telemetry.update();
        }
    }
}
