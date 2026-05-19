package org.austincans.lib.motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
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
public abstract class SparkIO<SparkType extends SparkBase, ConfigType extends SparkBaseConfig> extends MotorIOReal<MotorIOInputs> {
    protected final SparkType motor;
    protected final RelativeEncoder encoder;
    protected final SparkClosedLoopController controller;

    protected SparkIO(SparkType motor, ConfigType config) {
        super(motor.getDeviceId());
        this.motor = motor;
        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();
        motor.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.setConnected(true); // TODO: make this based on some property of the motor
        inputs.setPosition(Rotations.of(encoder.getPosition()));
        inputs.setVelocity(RPM.of(encoder.getVelocity()));
        inputs.setVoltage(Volts.of(motor.getBusVoltage()));
        inputs.setStatorCurrent(Amps.of(motor.getOutputCurrent()));
        inputs.setTemperature(Celsius.of(motor.getMotorTemperature()));
    }

    /** TODO: This should be removed. The only use  */
    public SparkType getMotor() {
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
        controller.setSetpoint(voltage.in(Volts), ControlType.kVoltage);
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

    /** TODO: This should probably be updated as part of outputs, or have a different method for updating configs */
    public void setEncoderPosition(double positionRotations) {
        encoder.setPosition(positionRotations);
    }
}
