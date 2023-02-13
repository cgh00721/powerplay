package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.io.Serializable;


public class SerializedPathPoint implements Serializable {
    byte[] gamepad1;
    byte[] gamepad2;
    Integer delayMs;

    private static final long serialVersionUID = 43L;

    public SerializedPathPoint(Gamepad gamepad1, Gamepad gamepad2, Integer delayMs) {
        try {
            this.gamepad1 = gamepad1.toByteArray();
            this.gamepad2 = gamepad2.toByteArray();
            this.delayMs = delayMs;
        } catch(Exception e) {

        }
    }
    // todo: write your code here
}