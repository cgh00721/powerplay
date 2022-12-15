package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;


public class LineFollower {
    public ColorSensor left_colorsensor;
    public ColorSensor right_colorsensor;
    public AllianceColor color;
    
    private final int TURN_THRESHOLD = 150;
    
    public LineFollower(ColorSensor left, ColorSensor right, AllianceColor color) {
        this.left_colorsensor = left;
        this.right_colorsensor = right;
        this.color = color;
    }
    
    public double getDifference() {
        return getLeftValue() - getRightValue();
    }
    
    public double getLeftValue() {
        if (this.color == AllianceColor.RED) {
            return left_colorsensor.red();
        } else if (this.color == AllianceColor.BLUE) {
            return left_colorsensor.blue();
        } else {
            return 0;
        }
    }
    
    public double getRightValue() {
        if (this.color == AllianceColor.RED) {
            return right_colorsensor.red();
        } else if (this.color == AllianceColor.BLUE) {
            return right_colorsensor.blue();
        } else {
            return 0;
        }
    }
    
    public double getLeftPower() {
        double lValue = getLeftValue();
        double rValue = getRightValue();
        double diff = lValue - rValue;
        if (diff > TURN_THRESHOLD) {
            return 0.25;
        } else if (diff < -TURN_THRESHOLD) {
            return -0.25;
        } else {
            return 0.25;
        }
    }
    
    public double getRightPower() {
        double lValue = getLeftValue();
        double rValue = getRightValue();
        double diff = lValue - rValue;
        if (diff > TURN_THRESHOLD) {
            return -0.25;
        } else if (diff < -TURN_THRESHOLD) {
            return 0.25;
        } else {
            return 0.25;
        }
    }
}
