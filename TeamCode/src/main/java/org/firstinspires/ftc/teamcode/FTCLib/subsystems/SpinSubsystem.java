package org.firstinspires.ftc.teamcode.FTCLib.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class SpinSubsystem extends SubsystemBase {

    private Motor spinMotor;

    public SpinSubsystem(Motor motor) {
        spinMotor = motor;
        spinMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }
    public void holdSpin() {
        spinMotor.stopMotor();
    }
}
