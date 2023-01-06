package org.firstinspires.ftc.teamcode.FTCLib.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;

public class ClawSubsystem extends SubsystemBase {

    private SimpleServo servo;

    public ClawSubsystem(SimpleServo servo) {
        this.servo = servo;
    }
    public void openClaw() {
        //servo.setPosition(openPos);
    }
    public void closeClaw() {
        //servo.setPosition(closePos);
    }
}
