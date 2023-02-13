package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.io.FileOutputStream;
import java.io.ObjectOutputStream;
import java.io.ObjectOutput;
import java.io.FileWriter;
public class GamepadRecorder {
    Gamepad gamepad1Cache;
    Gamepad gamepad2Cache;

    String filename;
    SerializedPath path;

    String delays = "";

    long time;

    public GamepadRecorder(String filename) {
        this.filename = filename;

        gamepad1Cache = new Gamepad();
        gamepad2Cache = new Gamepad();

        path = new SerializedPath();

        try {
            //
        } catch (Exception e) {
            GlobalTelemetry.telemetry.addLine(e.toString());
            GlobalTelemetry.telemetry.update();
            e.printStackTrace();
        }

        time = System.currentTimeMillis();
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        if (
                !isEqual(gamepad1, gamepad1Cache) ||
                        !isEqual(gamepad2, gamepad2Cache)
        ) {
            //return;
            write(gamepad1, gamepad2);
            try {
                gamepad1Cache.copy(gamepad1);
                gamepad2Cache.copy(gamepad2);
            } catch (Exception e) {
                GlobalTelemetry.telemetry.addLine(e.toString());
                GlobalTelemetry.telemetry.update();
                e.printStackTrace();
            }
        }
    }

    public void write(Gamepad gamepad1, Gamepad gamepad2) {
        String line = (System.currentTimeMillis() - time) + " " +
                gamepad1.left_stick_x + "f " +
                gamepad1.left_stick_y + "f " +
                gamepad1.left_bumper + " " +
                gamepad1.right_bumper + " " +
                gamepad1.a + " " +
                gamepad1.b + " " +
                gamepad1.x + " " +
                gamepad1.y + " " +
                gamepad1.left_trigger + "f " +
                gamepad1.right_trigger + "f " +
                gamepad1.dpad_down + " " +
                gamepad1.dpad_up + " " +
                gamepad1.dpad_left + " " +
                gamepad1.dpad_right + " " +

                gamepad2.left_stick_x + "f " +
                gamepad2.left_stick_y + "f " +
                gamepad2.left_bumper + " " +
                gamepad2.right_bumper + " " +
                gamepad2.a + " " +
                gamepad2.b + " " +
                gamepad2.x + " " +
                gamepad2.y + " " +
                gamepad2.left_trigger + "f " +
                gamepad2.right_trigger + "f " +
                gamepad2.dpad_down + " " +
                gamepad2.dpad_up + " " +
                gamepad2.dpad_left + " " +
                gamepad2.dpad_right;

        try {
            long new_time = System.currentTimeMillis();
            long timeDelta = new_time - time;
            delays += timeDelta + ",";
            time = new_time;
            path.add(gamepad1, gamepad2, (int)timeDelta);
        } catch (Exception e) {
            GlobalTelemetry.telemetry.addLine(e.toString());
            GlobalTelemetry.telemetry.update();
            e.printStackTrace();
        }

        //time = System.currentTimeMillis();
    }

    public void close() {
        try {
            FileOutputStream outputStream = new FileOutputStream(android.os.Environment.getExternalStorageDirectory().getPath() + "/FIRST/" + filename);
            ObjectOutputStream file = new ObjectOutputStream(outputStream);
            file.writeObject(path);
            file.flush();
            file.close();
            //System.out.print(1/0);
            GlobalTelemetry.telemetry.addLine(delays);
            GlobalTelemetry.telemetry.update();
        } catch (Exception e) {
            GlobalTelemetry.telemetry.addLine(e.toString());
            GlobalTelemetry.telemetry.update();
            e.printStackTrace();
        }
    }

    public boolean isEqual(Gamepad a, Gamepad b) {
        return a.left_stick_x == b.left_stick_x &&
                a.left_stick_y == b.left_stick_y &&
                a.right_stick_x == b.right_stick_x &&
                a.right_stick_y == b.right_stick_y &&
                a.left_trigger == b.left_trigger &&
                a.right_trigger == b.right_trigger &&
                a.left_bumper == b.left_bumper &&
                a.right_bumper == b.left_bumper &&
                a.dpad_up == b.dpad_up &&
                a.dpad_down == b.dpad_down &&
                a.dpad_left == b.dpad_left &&
                a.dpad_right == b.dpad_right &&
                a.a == b.a &&
                a.b == b.b &&
                a.x == b.x &&
                a.y == b.y;
    }
}