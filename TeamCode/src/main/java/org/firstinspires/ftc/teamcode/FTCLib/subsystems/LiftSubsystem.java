
package org.firstinspires.ftc.teamcode.FTCLib.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LiftSubsystem extends SubsystemBase {

    private Motor LiftMotor;

    public LiftSubsystem(Motor motor) {
        LiftMotor = motor;
    }
    public void motorUp() {
        LiftMotor.set(-0.7);
    }
    public void motorDown() {
        LiftMotor.set(0.3);
    }
    public void motorStop() {
        LiftMotor.stopMotor();
    }
}

