package org.austincans.lib.motors;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

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
    public void updateInputs(MotorIOInputs inputs) {
        inputs.connected = true; // TODO: make this based on some property of the motor
        inputs.position = Rotations.of(encoder.getPosition());
        inputs.velocity = RotationsPerSecond.of(encoder.getVelocity());
        inputs.voltage = Volts.of(motor.getBusVoltage());
        inputs.statorCurrentAmps = motor.getOutputCurrent();
        inputs.temperatureCelsius = motor.getMotorTemperature();
    }

    public Mtr getMotor(){
        return motor;
    }

    @Override
    public void applyOutputs(MotorIOOutputs outputs) {
        switch (outputs.mode) {
            case VOLTAGE -> setVoltage(outputs.voltage);
            case VELOCITY -> setVelocity(outputs.velocity);
            case POSITION -> setPosition(outputs.position);
        }
    }

    private void setVoltage(Voltage voltage) {
        controller.setSetpoint(voltage.baseUnitMagnitude(), ControlType.kVoltage);
    }

    private void setVelocity(AngularVelocity velocity) {
        controller.setSetpoint(
            velocity.in(RPM), 
            ControlType.kMAXMotionVelocityControl);
    }

    private void setPosition(Angle position) {
        controller.setSetpoint(
            position.in(Rotations), ControlType.kMAXMotionVelocityControl);
    }

    public void setEncoderPosition(double positionRotations) {
        encoder.setPosition(positionRotations);
    }
}
