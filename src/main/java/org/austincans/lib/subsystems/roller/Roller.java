package org.austincans.lib.subsystems.roller;

import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class Roller extends AdvancedSubsystem {
    private RollerIO io;
    private RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

    public Roller(RollerIO io) {
        this.io = io;
    }

    public void setVoltage(double voltage){
        io.setVoltage(voltage);
    }

    public void setVelocity(double velocityRPM) {
        io.setVelocity(velocityRPM);
    }

    @Override
    public void periodic() {
        Logger.processInputs(getName(), inputs);
        io.updateInputs(inputs);
    }

    @Override
    protected Command systemCheckCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'systemCheckCommand'");
    }

}
