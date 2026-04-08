package org.austincans.frc2026.subsystems.intake.roller;

import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
    @AutoLog
    public class RollerIOInputs {
        public double motorVoltage = 0.0;
        public double velocityRadPerSec = 0.0;
        public double temperatureCelsius = 0.0;
    }

    public class RollerIOOutputs {
        public double voltage;
    }

    public void updateInputs(RollerIOInputs inputs);
    
    public void applyOutputs(RollerIOOutputs outputs);

    public void registerSelfCheckHardware(AdvancedSubsystem subsystem);
}