package org.austincans.lib.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;

public class MotorIOTalonFXS extends MotorIOReal {
    private final TalonFXS motor;
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    private PositionVoltage positionVoltage = new PositionVoltage(0);

    protected MotorIOTalonFXS(int deviceId, String canbus, TalonFXConfiguration motorConfig, PIDFFConfig pidffConfig,
            SoftLimitConfig softLimitConfig, NeutralModeValue nuetralMode, InvertedValue isInverted) {
        super(deviceId);
        motor = new TalonFXS(deviceId, new CANBus(canbus));
        motorConfig.Slot0.kP = pidffConfig.kP();
        motorConfig.Slot0.kI = pidffConfig.kI();
        motorConfig.Slot0.kD = pidffConfig.kD();
        motorConfig.Slot0.kS = pidffConfig.kS();
        motorConfig.Slot0.kG = pidffConfig.kG();
        motorConfig.Slot0.kV = pidffConfig.kV();
        motorConfig.Slot0.kA = pidffConfig.kA();
        motorConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(softLimitConfig.forwardSoftLimitEnabled());
        motorConfig.SoftwareLimitSwitch.withForwardSoftLimitThreshold(softLimitConfig.forwardSoftLimit());
        motorConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(softLimitConfig.reverseSoftLimitEnabled());
        motorConfig.SoftwareLimitSwitch.withReverseSoftLimitThreshold(softLimitConfig.reverseSoftLimit());
        motorConfig.MotorOutput.withNeutralMode(nuetralMode);
        motorConfig.MotorOutput.withInverted(isInverted);
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.positionRad = motor.getPosition().getValue().in(Radians);    
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
        inputs.velocityRadPerSec = motor.getVelocity().getValue().in(RadiansPerSecond);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        motor.setControl(velocityVoltage.withVelocity(AngularVelocity.ofBaseUnits(velocityRadPerSec, RadiansPerSecond)));
    }

    @Override
    public void setPosition(double positionRad) {
        motor.setControl(positionVoltage.withPosition(positionRad));
    }
}
