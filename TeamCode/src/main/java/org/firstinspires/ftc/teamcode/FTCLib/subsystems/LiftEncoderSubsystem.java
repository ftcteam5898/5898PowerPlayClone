package org.firstinspires.ftc.teamcode.FTCLib.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.*;

public class LiftEncoderSubsystem {
    private Motor liftMotor;

    public LiftEncoderSubsystem(Motor motor) {
        liftMotor = motor;
        //liftMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int pos = liftMotor.getCurrentPosition();
        double vel = liftMotor.getCorrectedVelocity();
        Motor.Encoder liftEncoder = liftMotor.encoder;
        double revolutions = liftEncoder.getRevolutions();
        liftEncoder.setDistancePerPulse(2.0);
        double distance = liftEncoder.getDistance();
        //liftMotor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftMotor.motor.setTargetPosition(10);
        //liftMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
    public void motorDownSlow() {
        liftMotor.set(0.2);
        liftMotor.setTargetPosition(liftMotor.getCurrentPosition() +200);
    }
    public void resetMotor() {
        liftMotor.resetEncoder();
        /*
        liftMotor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.motor.setTargetPosition(10);
        liftMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         */
    }
    public void motorHold() {
        /*
        int pos = liftMotor.getCurrentPosition();
        liftMotor.motor.setTargetPosition(pos);
        liftMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         */
        liftMotor.set(-0.0004);
    }
    public void motorStop() {
        liftMotor.stopMotor();
    }
}
