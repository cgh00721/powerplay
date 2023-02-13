package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class RecorderTeleop extends LinearOpMode {

    public void runOpMode() {
        GlobalTelemetry.telemetry = telemetry;
        GamepadRecorder recorder = new GamepadRecorder("Record7959_test_serialized.txt");
        GamepadRobot robot = new GamepadRobot(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            robot.handleGamepad(gamepad1, gamepad2);
            recorder.update(gamepad1, gamepad2);
            if (gamepad1.y) {
                telemetry.addLine(recorder.delays);
                telemetry.update();
                while (opModeIsActive()) {}
                break;
            }
        }

        recorder.close();
    }
}