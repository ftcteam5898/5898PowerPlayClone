package org.firstinspires.ftc.teamcode.FTCLib.subsystems;

import com.arcrobotics.ftclib.hardware.SensorColor;

public class ColorSubsystem {
    private SensorColor lawley;
    private int blueVal;
    private int redVal;

    public ColorSubsystem(SensorColor sensor) {
        lawley = sensor;
    }
    public void readColor() {
        blueVal = lawley.blue();
        redVal = lawley.red();

    }
}
