
package org.firstinspires.ftc.teamcode.FTCLib.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;

public class ClawSubsystem extends SubsystemBase {

    private SimpleServo servo;

    public ClawSubsystem(SimpleServo servo) {
        this.servo = servo;
    }
    public void openClaw() {
        servo.setPosition(-60.0);
    }
    public void closeClaw() {
        servo.setPosition(50.0);
    }
    public void resetClaw() {
        servo.setPosition(0.0);
    }
}

