package org.austincans.lib.subsystems.roller;

import java.util.List;

import org.austincans.lib.motors.MotorIO.MotorIOInputs;
import org.austincans.lib.subsystems.selfcheck.HardwareFault;
import org.austincans.lib.subsystems.selfcheck.SelfChecking;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
    @AutoLog
    public class RollerIOInputs {
        public MotorIOInputs motorInputs;
    }

    public void setVelocity(double velocityRPM);

    public void updateInputs(RollerIOInputs inputs);

    public void setVoltage(double voltage);

    public default List<HardwareFault> checkForFaults() {
        return List.of();
    }

    public default List<SelfChecking> getHardware() {
        return List.of();
    }
}
