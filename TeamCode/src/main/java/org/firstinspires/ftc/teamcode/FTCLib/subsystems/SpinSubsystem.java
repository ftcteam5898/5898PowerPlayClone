package org.firstinspires.ftc.teamcode.FTCLib.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class SpinSubsystem extends SubsystemBase {

    private Motor spinMotor;

    public SpinSubsystem(Motor motor) {
        spinMotor = motor;
    }
    public void holdSpin() {
        spinMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        spinMotor.stopMotor();
    }
    public void rotateRight() {
        spinMotor.set(0.5);
    }
    public void rotateLeft() {
        spinMotor.set(-0.5);
    }
}
