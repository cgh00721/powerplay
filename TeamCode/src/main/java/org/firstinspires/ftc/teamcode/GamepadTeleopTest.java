package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class GamepadTeleopTest extends LinearOpMode {

    public void runOpMode() {
        //GamepadRecorder recorder = new GamepadRecorder("Record7959.txt");
        GamepadRobot robot = new GamepadRobot(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            robot.handleGamepad(gamepad1, gamepad2);
            //recorder.update(gamepad1, gamepad2);
        }
    }
}