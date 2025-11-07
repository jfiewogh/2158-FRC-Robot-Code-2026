// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// oooooooooooo ooooooooo.     .oooooo.        ooooooooo.              .o8                     .
// `888'     `8 `888   `Y88.  d8P'  `Y8b       `888   `Y88.           "888                   .o8
//  888          888   .d88' 888                888   .d88'  .ooooo.   888oooo.   .ooooo.  .o888oo
//  888oooo8     888ooo88P'  888                888ooo88P'  d88' `88b  d88' `88b d88' `88b   888
//  888    "     888`88b.    888                888`88b.    888   888  888   888 888   888   888
//  888          888  `88b.  `88b    ooo        888  `88b.  888   888  888   888 888   888   888 .
// o888o        o888o  o888o  `Y8bood8P'       o888o  o888o `Y8bod8P'  `Y8bod8P' `Y8bod8P'   "888"
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
//
// Updated on Wednesday November 05, 2025 at 18:44:58 CST

package org.austincans.frc2158.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public ModuleIOData data =
                new ModuleIOData(false, 0, 0, 0, 0, 0, false, false, Rotation2d.kZero, Rotation2d.kZero, 0, 0, 0, 0);

        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    public record ModuleIOData(
            boolean driveConnected,
            double drivePositionRad,
            double driveVelocityRadPerSec,
            double driveAppliedVolts,
            double driveSupplyCurrentAmps,
            double driveTorqueCurrentAmps,
            boolean turnConnected,
            boolean turnEncoderConnected,
            Rotation2d turnAbsolutePosition,
            Rotation2d turnPosition,
            double turnVelocityRadPerSec,
            double turnAppliedVolts,
            double turnSupplyCurrentAmps,
            double turnTorqueCurrentAmps) {}

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /** Run the drive motor at the specified open loop value. */
    public default void runDriveOpenLoop(double output) {}

    /** Run the turn motor at the specified open loop value. */
    public default void runTurnOpenLoop(double output) {}

    /** Run the drive motor at the specified velocity. */
    public default void runDriveVelocity(double velocityRadPerSec, double feedforward) {}

    /** Run the turn motor to the specified rotation. */
    public default void runTurnPosition(Rotation2d rotation) {}

    /** Set P, I, and D gains for closed loop control on drive motor. */
    public default void setDrivePID(double kP, double kI, double kD) {}

    /** Set P, I, and D gains for closed loop control on turn motor. */
    public default void setTurnPID(double kP, double kI, double kD) {}

    /** Set brake mode on drive motor */
    public default void setBrakeMode(boolean enabled) {}
}
