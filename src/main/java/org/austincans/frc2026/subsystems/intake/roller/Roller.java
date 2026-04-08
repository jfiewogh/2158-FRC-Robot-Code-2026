package org.austincans.frc2026.subsystems.intake.roller;

import org.austincans.frc2026.Constants;
import org.austincans.frc2026.subsystems.intake.roller.RollerIO.RollerIOOutputs;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Roller extends AdvancedSubsystem {
    private final RollerIO io;
    private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
    private final RollerIOOutputs outputs = new RollerIOOutputs();

    /** Should be not running on start */
    @AutoLogOutput public boolean running = false;

    public Roller(RollerIO io) {
        this.io = io;
        io.registerSelfCheckHardware(this);
        registerCheckCommand(systemCheckCommand());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Roller", inputs);
    }

    @Override
    public void periodicAfterScheduler() {
        io.applyOutputs(outputs);
    }

    public void intake() {
        running = true;
        outputs.voltage = 0.35 * Constants.maxVoltage;
    }

    public void outtake() {
        running = true;
        outputs.voltage = -0.35 * Constants.maxVoltage;
    }

    public void stop() {
        running = false;
        outputs.voltage = 0.0;
    }

    public void toggle() {
        if (running) {
            stop();
        } else {
            intake();
        }
    }

    public Command outtakeCommand() {
        return Commands.runOnce(this::outtake, this);
    }

    public Command toggleCommand() {
        return Commands.runOnce(this::toggle, this);
    }

    @Override
    protected Command systemCheckCommand() {
        return Commands.none();
    }
}
