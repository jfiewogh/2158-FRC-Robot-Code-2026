package org.austincans.lib.subsystems.drive;

import org.austincans.lib.configs.MotorConfig;
import org.austincans.lib.configs.SwerveModuleConfig;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/* Wrapper class for ModuleIO */
public class Module<DriveMtrCnfg extends MotorConfig, SteerMtrCnfg extends MotorConfig> extends AdvancedSubsystem {
    private final ModuleIO io;
    private final ModuleIORecordsAutoLogged records = new ModuleIORecordsAutoLogged();
    private final ModuleIORequestsAutoLogged requests = new ModuleIORequestsAutoLogged();
    private final int index;
    private final double wheelRadiusMeters;
    private double driveSpeedMultiplier;

    // Connected debouncers
    private final Debouncer driveMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer turnMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer turnEncoderConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    // Connection alerts
    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert turnEncoderDisconnectedAlert;

    protected Module(ModuleIO io, int index, SwerveModuleConfig<DriveMtrCnfg, SteerMtrCnfg> moduleConfig) {
        this.io = io;
        this.index = index;
        wheelRadiusMeters = moduleConfig.wheelRadiusMeters();
        driveSpeedMultiplier = moduleConfig.driveSpeedMultiplier();

        driveDisconnectedAlert = new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
        turnDisconnectedAlert = new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
        turnEncoderDisconnectedAlert = new Alert(
            "Disconnected turn encoder on module " + Integer.toString(index) + ".",
            AlertType.kError);
    }

    @Override
    public void periodic() {
        io.updateRecords(records);
        Logger.processInputs("Module/Records", records);

        // Update alerts
        driveDisconnectedAlert.set(
            !driveMotorConnectedDebouncer.calculate(records.driveConnected) && !Robot.isJITing());
        turnDisconnectedAlert.set(
            !turnMotorConnectedDebouncer.calculate(records.turnConnected) && !Robot.isJITing());
        turnEncoderDisconnectedAlert.set(
            !turnEncoderConnectedDebouncer.calculate(records.turnEncoderConnected) && !Robot.isJITing());

        io.applyRequests(requests);
        Logger.processInputs("Module/Requests", requests)
    }

    /**
     * Runs the module with the specified setpoint state.
     * Mutates the state to optimize it.
     */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize velocity setpoint
        Rotation2d currentSteerAngle = getSteerAngle();
        state.optimize(currentSteerAngle);
        state.cosineScale(currentSteerAngle);

        // Apply setpoints
        double driveVelocityRadPerSec = state.speedMetersPerSecond / wheelRadiusMeters;
        requests.setDriveVelocity(driveVelocityRadPerSec * driveSpeedMultiplier);
        requests.setSteerAngle(state.angle);
    }

    /**
     * Runs the module with the specified output while controlling to zero degrees.
     */
    public void runCharacterization(double output) {
        requests.setDriveOpenLoop(output);
        requests.setSteerAngle(Rotation2d.kZero);
    }

    /** Runs the module with the specified output toward Angular Motions */
    public void runAngularCharacterization(double output, Translation2d moduleLocationMeters) {
        requests.setDriveOpenLoop(output);
        requests.setSteerAngle(
                new Rotation2d(moduleLocationMeters.getX(), moduleLocationMeters.getY())
                        .plus(Rotation2d.kCCW_Pi_2));
    }

    /** Disables all outputs to motors. */
    public void stop() {
        requests.setDriveOpenLoop(0);
        requests.setSteerOpenLoop(0);
    }

    private Rotation2d getSteerAngle() {
        return Rotation2d.fromRadians(records.steerAbsoluteEncoderInputs.absolutePositionRad);
    }

    /** Returns the current drive position of the module in meters. */
    private double getDrivePositionMeters() {
        return records.driveMotorInputs.positionRad * wheelRadiusMeters;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    private double getDriveVelocityMetersPerSec() {
        return records.driveMotorInputs.velocityRadPerSec * wheelRadiusMeters;
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), getSteerAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSec(), getSteerAngle());
    }

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition() {
        return records.driveMotorInputs.positionRad;
    }

    /** Returns the module velocity in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(records.driveMotorInputs.velocityRadPerSec);
    }

    public void setDriveSpeedMultiplier(double driveSpeedMultiplier) {
        this.driveSpeedMultiplier = driveSpeedMultiplier;
    }

    public Command systemCheckCommand() {
        return Commands.sequence(
                Commands.runOnce(
                        () -> {
                            clearFaults();
                            stop();
                            requests.steerMotorRequests.setVelocity(20);
                        },
                        this),
                Commands.waitSeconds(0.3),
                Commands.runOnce(
                        () -> {
                            if (Units.radiansToDegrees(records.turnVelocityRadPerSec) < 20) {
                                addSubsystemFault(
                                        "SWRV" + String.valueOf(index),
                                        "[System Check] Rotation motor encoder velocity measured too slow",
                                        false);
                            }
                        },
                        this),
                Commands.run(
                        () -> {
                            requests.setSteerAngle(Rotation2d.fromDegrees(90));
                        },
                        this)
                        .withTimeout(1.0),
                Commands.runOnce(
                        () -> {
                            if (records.turnAbsolutePosition.getDegrees() < 70
                                    || records.turnAbsolutePosition.getDegrees() > 110) {
                                addSubsystemFault(
                                        "SWRV" + String.valueOf(index),
                                        "[System Check] Rotation Motor did not reach target position",
                                        false);
                            }
                        },
                        this),
                Commands.runOnce(
                        () -> {
                            requests.setDriveVelocity(Units.degreesToRadians(90));
                        },
                        this),
                Commands.waitSeconds(0.5),
                Commands.runOnce(
                        () -> {
                            if (Units.radiansToDegrees(records.driveVelocityRadPerSec) < 0.25) {
                                addSubsystemFault(
                                        "SWRV" + String.valueOf(index),
                                        "[System Check] Drive motor encoder velocity too slow",
                                        false);
                            }
                            requests.setDriveVelocity(0);
                        },
                        this),
                Commands.waitSeconds(0.25))
                .andThen(Commands.runOnce(this::stop, this));
    }
}
