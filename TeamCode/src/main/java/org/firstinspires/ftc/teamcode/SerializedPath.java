package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.io.Serializable;
import com.qualcomm.robotcore.hardware.Gamepad;


public class SerializedPath implements Serializable {
    private static final long serialVersionUID = 42L;
    // todo: write your code here

    ArrayList<SerializedPathPoint> points;

    public SerializedPath() {
        points = new ArrayList<SerializedPathPoint>();
    }

    public void add(Gamepad gamepad1, Gamepad gamepad2, int delayMs) {
        this.points.add(new SerializedPathPoint(gamepad1, gamepad2, delayMs));
    }
}