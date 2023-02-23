
package org.firstinspires.ftc.teamcode.FTCLib.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LiftSubsystem extends SubsystemBase {

    private Motor liftMotor;

    public LiftSubsystem(Motor motor) {
        liftMotor = motor;
    }
    public LiftSubsystem(Motor motor, int different) {
        liftMotor = motor;
        liftMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void motorUp() {
        liftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        liftMotor.set(-0.7);
    }
    public void motorDown() {
        liftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        liftMotor.set(0.3);
    }
    public void highMotorHold() {
        liftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        liftMotor.set(-0.3);
    }
    public void motorHold() {
        liftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        liftMotor.stopMotor();
    }
    public void motorStop() {
        liftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        liftMotor.stopMotor();
    }
}

