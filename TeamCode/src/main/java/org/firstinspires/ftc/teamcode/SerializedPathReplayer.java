package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.io.ObjectInputStream;
import java.io.FileInputStream;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*public class SerializedPathReplayer extends LinearOpMode { // extend LinearOpMode so we have access to sleep()

    // todo: write your code here
    public static void runPath(GamepadRobot robot, String filename, LinearOpMode opMode) {
        Object o;
        ObjectInputStream file;
        try {
        FileInputStream fileInput = new FileInputStream(android.os.Environment.getExternalStorageDirectory().getPath() + "/FIRST/" + filename);
        file = new ObjectInputStream(fileInput);

        o = file.readObject();

        GlobalTelemetry.telemetry.addData(">", (SerializedPath)o);
        GlobalTelemetry.telemetry.update();

        file.close();

        //waitForStart();

        SerializedPath path = (SerializedPath)o;

        //GamepadRobot robot = new GamepadRobot(hardwareMap);

        Gamepad gamepad1 = new Gamepad();
        Gamepad gamepad2 = new Gamepad();

        long delay = System.currentTimeMillis();
        long gamepadDelay = 0;

        String delays = "";

        for (int i = 0; i < path.points.size(); i++) {

            gamepad1.fromByteArray(path.points.get(i).gamepad1);
            gamepad2.fromByteArray(path.points.get(i).gamepad2);
            GlobalTelemetry.telemetry.addLine(path.points.get(i).delayMs.toString()+ ", 0");
            GlobalTelemetry.telemetry.update();
            delay = System.currentTimeMillis();
            opMode.sleep(path.points.get(i).delayMs);
            gamepadDelay = System.currentTimeMillis();
            robot.handleGamepad(gamepad1, gamepad2);
            gamepadDelay = System.currentTimeMillis() - gamepadDelay;
            if (i > 0) // to tell if the GamepadRobot code is taking too long and throwing off the accuracy of the timing
                GlobalTelemetry.telemetry.addLine("" + (System.currentTimeMillis() - delay) + " " + path.points.get(i-1).delayMs + " " + gamepadDelay);
            //while(opModeIsActive()){}
            delays += path.points.get(i).delayMs.toString()+ ",";
        }

        //telemetry.addLine(delays);
        GlobalTelemetry.telemetry.update();
        //while (true) {}
        //while (opModeIsActive()) {}

        } catch(Exception e) {
            GlobalTelemetry.telemetry.addLine(e.toString() + ", 1");
            GlobalTelemetry.telemetry.update();
            //waitForStart();
            //while(opModeIsActive()){}
        }

    }

    public void runOpMode() {}

}*/

// Utility to replay paths serialized to files
public class SerializedPathReplayer{

    public static void runPath(GamepadRobot robot, String filename, OpMode opMode) {
        try {
            runPathInternal(robot, filename, opMode);
        } catch (Exception e) {
            opMode.telemetry.addLine(e.toString());
            opMode.telemetry.update();
            e.printStackTrace();
        }
    }

    // replay a path given a GamepadRobot, name of file in /sdcard/FIRST/ with serialized path, and an OpMode (for telemetry)
    public static void runPathInternal(GamepadRobot robot, String filename, OpMode opMode) throws Exception {
        // open file to read objects
        FileInputStream fileStream = new FileInputStream(android.os.Environment.getExternalStorageDirectory().getPath() + "/FIRST/" + filename);
        ObjectInputStream objectStream = new ObjectInputStream(fileStream);

        // read a generic Object from the file, and cast it back to SerializedPath
        SerializedPath path = (SerializedPath)(objectStream.readObject());

        // close the file
        objectStream.close();
        fileStream.close();

        // create fake gamepads to replay inputs through
        Gamepad gamepad1 = new Gamepad();
        Gamepad gamepad2 = new Gamepad();

        // will hold the maximum number of milliseconds taken by a call to robot.handleGamepad
        int maxGamepadHandleTime = Integer.MIN_VALUE;

        // will hold the maximum number of milliseconds taken loading gamepads from a file
        int maxGamepadLoadTime = Integer.MIN_VALUE;

        int prevTime = 0;

        // loop through all points in the recording where input changed on a gamepad
        for (int i = 0; i < path.points.size(); i++) {
            // get the object representing the changed point
            SerializedPathPoint point = path.points.get(i);

            // delay for the stored amount of time
            int delay = Math.max((int)point.delayMs - (int)(prevTime*2), 0);
            /*TimeUnit.MILLISECONDS*/((LinearOpMode)opMode).sleep(delay);
            opMode.telemetry.addData("sleepTime corrected: ", delay);

            opMode.telemetry.addData("gamepadTime: ", prevTime);

            // load the gamepad states from the byte arrays in the file, and time how long this take
            long time = System.currentTimeMillis();
            gamepad1.fromByteArray(point.gamepad1);
            gamepad2.fromByteArray(point.gamepad2);
            time = System.currentTimeMillis() - time;

            // if this is the new longest it's taken to load the gamepads back from the byte arrays,
            // store it as the longest time
            if (time > maxGamepadLoadTime) {
                maxGamepadLoadTime = (int)time;
            }

            // update the inputs sent to the GamepadRobot, and time how long it takes
            time = System.currentTimeMillis();
            robot.handleGamepad(gamepad1, gamepad2);
            time = System.currentTimeMillis() - time;
            prevTime = (int)time;

            opMode.telemetry.addData("prevTime: ", prevTime);

            // if this is the new longest it's taken to handle gamepads, update the maximum time taken
            if (time > maxGamepadHandleTime) {
                maxGamepadHandleTime = (int)time;
            }

            // show telemetry for debugging purposes
            opMode.telemetry.addData("Max handleGamepad time: ", maxGamepadHandleTime + "ms");
            opMode.telemetry.addData("Max gamepad loading time: ", maxGamepadLoadTime + "ms");
            opMode.telemetry.addData("Step number: ", (i + 1) + " of " + path.points.size());
            opMode.telemetry.update();
        }
    }
}