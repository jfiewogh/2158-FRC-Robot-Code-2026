package org.austincans.lib.motors;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.util.Units;

/*
 * To configure while deployed, use kNoResetSafeParameters and kNoPersistParameters 
 * so that you can configure only specific values while keeping existing ones.
 */
public abstract class MotorIOSpark<Mtr extends SparkBase, Cnfg extends SparkBaseConfig> extends MotorIOReal {
    protected final Mtr motor;
    protected final RelativeEncoder encoder;
    protected final SparkClosedLoopController controller;

    protected MotorIOSpark(Mtr motor, Cnfg config) {
        super(motor.getDeviceId());
        this.motor = motor;
        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();
        motor.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateRecords(MotorIORecords records) {
        records.connected = true; // TODO: make this based on some property of the motor
        records.positionRad = Units.rotationsToRadians(encoder.getPosition());
        records.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
        records.appliedVolts = motor.getBusVoltage();
        records.statorCurrentAmps = motor.getOutputCurrent();
        records.temperatureCelsius = motor.getMotorTemperature();
    }

    @Override
    public void applyRequests(MotorIORequests requests) {
        switch (requests.requestMode) {
            case VOLTAGE -> setVoltage(requests.voltage);
            case VELOCITY -> setVelocity(requests.velocityRadPerSec);
            case POSITION -> setPosition(requests.positionRad);
        }
    }

    private void setVoltage(double voltage) {
        controller.setSetpoint(voltage, ControlType.kVoltage);
    }

    private void setVelocity(double velocityRadPerSec) {
        controller.setSetpoint(
            Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec), 
            ControlType.kMAXMotionVelocityControl);
    }

    private void setPosition(double positionRad) {
        controller.setSetpoint(
            Units.radiansToRotations(positionRad), ControlType.kMAXMotionVelocityControl);
    }

    public void setEncoderPosition(double positionRotations) {
        encoder.setPosition(positionRotations);
    }
}
