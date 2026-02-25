package org.austincans.frc2158.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public abstract class TurretIOReal implements TurretIO {
    protected final CANcoder encoder;

    public TurretIOReal() {
        encoder = new CANcoder(TurretConstants.turretEncoderId, TurretConstants.turretCANBus);
        
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoder.getConfigurator().apply(encoderConfig);
        
        encoder.setPosition(0);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.positionRad = encoder.getPosition().getValue().in(Radians);
        inputs.velocityRadPerSec = encoder.getVelocity().getValue().in(RadiansPerSecond);
    }
}
