
package org.firstinspires.ftc.teamcode.FTCLib.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LiftSubsystem extends SubsystemBase {

    private Motor motor;

    public LiftSubsystem(Motor motor) {
        this.motor = (Motor) motor;
    }
    public void motorUp() {
        motor.set(-0.7);
    }
    public void motorDown() {
        motor.set(0.3);
    }
    public void motorStop() {
        motor.stopMotor();
    }
}

