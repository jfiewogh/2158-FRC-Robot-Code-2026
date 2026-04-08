package org.austincans.frc2026.subsystems.intake.roller;

import org.austincans.frc2026.subsystems.intake.IntakeConstants;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.util.Units;

public class RollerIOSparkFlex implements RollerIO {
    private final SparkFlex motor;
    private final RelativeEncoder encoder;

    public RollerIOSparkFlex() {
        motor = new SparkFlex(IntakeConstants.rollerMotorId, MotorType.kBrushless);
        encoder = motor.getEncoder();
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.motorVoltage = motor.getBusVoltage();
        inputs.temperatureCelsius = motor.getMotorTemperature();
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
    }

    @Override
    public void applyOutputs(RollerIOOutputs outputs) {
        motor.setVoltage(outputs.voltage);
    }

    @Override
    public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
        subsystem.registerHardware("Roller Motor", motor);
    }
    
}
