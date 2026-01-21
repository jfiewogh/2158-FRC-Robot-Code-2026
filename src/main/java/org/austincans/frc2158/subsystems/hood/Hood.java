package org.austincans.frc2158.subsystems.hood;

import org.austincans.frc2158.subsystems.hood.HoodIO.hoodIOInputs;
import org.austincans.frc2158.subsystems.selfcheck.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

import org.austincans.frc2158.math.Angle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends AdvancedSubsystem {
    private HoodIO io;
    private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private Angle angle;

    Command runHoodMotorCommand(Angle angle) {
        return Commands.run((() -> io.setAngle(angle)));
    }

    public void setAngle(Rotation2d angle) {
        io.setAngleRotation2d(angle);
    }

    public void setAngle(Angle angle){
        io.setAngle(angle);
    }

    public Angle getAngle() {
        io.getAngle();
    }

    public void runHoodVoltage(double volts) {
        io.applyVolts(volts);
    }

    public Command stopCommand() {
        return new InstantCommand(
                () -> {
                    io.runHoodMotor(0);
                });
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
    }

    public Command systemCheckCommand() {
        return Commands.sequence(
                Commands.runOnce(
                        () -> {
                            clearFaults();
                            io.applyVolts(0);
                            io.setAngle(Angle.fromDegrees(10));
                        },
                        this),
                Commands.waitSeconds(0.1),
                Commands.runOnce(
                        () -> {
                            if (inputs.hoodRadsPerSec < 1) {
                                addFault(
                                        "[System Check] Hood encoder velocity measured too slow",
                                        false,
                                        true);
                            }
                        },
                        this),
                Commands.run(
                        () -> {
                            io.setAngle(Angle.fromDegrees(20));
                        },
                        this)
                        .withTimeout(1.0),
                Commands.runOnce(
                        () -> {
                            if (inputs.hoodAngleDegrees < 22 || inputs.hoodAngleDegrees > 18) {
                                addFault(
                                        "[System Check] Hood did not reach target position", false, true);
                            }
                        },
                        this),
                Commands.waitSeconds(0.25))
                .until(() -> getFaults().size() > 0)
                .andThen(Commands.runOnce(this::stopCommand, this));
    }
}
