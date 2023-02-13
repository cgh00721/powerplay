package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class ReplayTestAuton extends LinearOpMode {

    // todo: write your code here
    public void runOpMode() {
        GlobalTelemetry.telemetry = telemetry;
        GamepadRobot robot = new GamepadRobot(hardwareMap);
        waitForStart();
        SerializedPathReplayer.runPath(robot, "Record7959_test_serialized.txt", this);
        while (opModeIsActive()) {}
    }
}