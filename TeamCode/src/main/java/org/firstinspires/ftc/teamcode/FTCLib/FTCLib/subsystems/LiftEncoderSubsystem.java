package org.firstinspires.ftc.teamcode.FTCLib.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.*;

public class LiftEncoderSubsystem {
    private Motor liftMotor;

    public LiftEncoderSubsystem(Motor motor) {
        liftMotor = motor;
        int pos = liftMotor.getCurrentPosition();
        double vel = liftMotor.getCorrectedVelocity();
        Motor.Encoder liftEncoder = liftMotor.encoder;
        double revolutions = liftEncoder.getRevolutions();
        liftEncoder.setDistancePerPulse(2.0);
        double distance = liftEncoder.getDistance();
    }
    public com.arcrobotics.ftclib.hardware.motors.Motor getLiftMotor() {
        return liftMotor;
    }

    public void motorUp() {
            liftMotor.set(-0.7);
            liftMotor.setTargetPosition(liftMotor.getCurrentPosition() -200);
    }
    public void motorUpSlow() {
        liftMotor.set(-0.2);
        liftMotor.setTargetPosition(liftMotor.getCurrentPosition() -200);
    }
    public void motorDown() {
        liftMotor.set(0.7);
        liftMotor.setTargetPosition(liftMotor.getCurrentPosition() + 200);
    }
    public void allTheWayDown() {
        liftMotor.set(0.7);
        liftMotor.setTargetPosition(liftMotor.getCurrentPosition() + liftMotor.getCurrentPosition() * -1);
    }
    public void motorDownSlow() {
        liftMotor.set(0.2);
        liftMotor.setTargetPosition(liftMotor.getCurrentPosition() +200);
    }
    public void resetMotor() {
        liftMotor.resetEncoder();
    }
    public void motorHold() {
        if (liftMotor.getCurrentPosition() <= -1000) {
            liftMotor.set(-0.0004);
        }
        else if (liftMotor.getCurrentPosition() <= -2000) {
            liftMotor.set(-0.0005);
        }
    }
    public void motorStop() {
        liftMotor.stopMotor();
    }
}
