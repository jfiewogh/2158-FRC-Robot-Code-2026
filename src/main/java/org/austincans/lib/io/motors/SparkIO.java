package org.austincans.lib.io.motors;

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
public abstract class SparkIO<SparkType extends SparkBase, ConfigType extends SparkBaseConfig> extends BaseMotorIO<MotorIO.MotorIOInputs> {
    final SparkType motor;
    final RelativeEncoder encoder;
    final SparkClosedLoopController controller;

    SparkIO(SparkType motor, ConfigType config) {
        this.motor = motor;
        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();
        motor.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.setPosition(Rotations.of(encoder.getPosition()));
        inputs.setVelocity(RPM.of(encoder.getVelocity()));
        inputs.setVoltage(Volts.of(motor.getBusVoltage()));
        inputs.setStatorCurrent(Amps.of(motor.getOutputCurrent()));
        inputs.setTemperature(Celsius.of(motor.getMotorTemperature()));
    }

    @Override
    void setVoltage(Voltage voltage) {
        controller.setSetpoint(voltage.in(Volts), ControlType.kVoltage);
    }

    @Override
    void setVelocity(AngularVelocity velocity) {
        controller.setSetpoint(
            velocity.in(RPM), 
            ControlType.kMAXMotionVelocityControl);
    }

    @Override
    void setPosition(Angle position) {
        controller.setSetpoint(
            position.in(Rotations), ControlType.kMAXMotionVelocityControl);
    }

    /** TODO: This should probably be updated as part of outputs, or have a different method for updating configs */
    public void setEncoderPosition(double positionRotations) {
        encoder.setPosition(positionRotations);
    }
}
