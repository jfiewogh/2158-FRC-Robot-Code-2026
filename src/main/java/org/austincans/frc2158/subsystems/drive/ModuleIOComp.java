// Copyright (c) 2026 the ausTIN CANs
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
// Updated on Saturday January 31, 2026 at 12:05:30 CST

package org.austincans.frc2158.subsystems.drive;

import static org.austincans.frc2158.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import org.austincans.frc2158.subsystems.drive.DriveConstants.ModuleConfig;
import org.austincans.frc2158.util.PhoenixUtil;

public class ModuleIOComp implements ModuleIO {
  // Hardware objects
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder encoder;

  // Config
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration turnConfig = new TalonFXConfiguration();
  private final Rotation2d encoderOffset;
  private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(8);

  // Control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0);

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveSupplyCurrentAmps;
  private final StatusSignal<Current> driveStatorCurrentAmps;
  private final StatusSignal<Current> driveTorqueCurrentAmps;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnSupplyCurrentAmps;
  private final StatusSignal<Current> turnStatorCurrentAmps;
  private final StatusSignal<Current> turnTorqueCurrentAmps;

  public ModuleIOComp(ModuleConfig config) {
    driveTalon = new TalonFX(config.driveMotorId(), DriveConstants.moduleCANBus);
    turnTalon = new TalonFX(config.turnMotorId(), DriveConstants.moduleCANBus);
    encoder = new CANcoder(config.encoderChannel(), DriveConstants.moduleCANBus);
    encoderOffset = config.encoderOffset();

    // Configure drive motor
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.driveReduction;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = DriveConstants.driveCurrentLimitAmps;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -DriveConstants.driveCurrentLimitAmps;
    driveConfig.CurrentLimits.StatorCurrentLimit = DriveConstants.driveCurrentLimitAmps;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    turnConfig.Feedback.FeedbackRemoteSensorID = config.encoderChannel();
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnConfig.Feedback.RotorToSensorRatio = DriveConstants.turnReduction;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = DriveConstants.turnCurrentLimitAmps;
    turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -DriveConstants.turnCurrentLimitAmps;
    turnConfig.CurrentLimits.StatorCurrentLimit = DriveConstants.turnCurrentLimitAmps;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.MotorOutput.Inverted =
        config.turnInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

    // Configure CANCoder
    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = config.encoderOffset().getRotations();
    cancoderConfig.MagnetSensor.SensorDirection =
        config.encoderInverted()
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> encoder.getConfigurator().apply(cancoderConfig));

    // Create drive status signals
    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getPosition().clone());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveSupplyCurrentAmps = driveTalon.getSupplyCurrent();
    driveStatorCurrentAmps = driveTalon.getStatorCurrent();
    driveTorqueCurrentAmps = driveTalon.getTorqueCurrent();

    // Create turn status signals
    turnAbsolutePosition = encoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon.getPosition().clone());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnSupplyCurrentAmps = turnTalon.getSupplyCurrent();
    turnStatorCurrentAmps = turnTalon.getStatorCurrent();
    turnTorqueCurrentAmps = turnTalon.getTorqueCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        DriveConstants.odometryFrequency, drivePosition, turnPosition, turnAbsolutePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrentAmps,
        driveStatorCurrentAmps,
        driveTorqueCurrentAmps,
        turnVelocity,
        turnAppliedVolts,
        turnSupplyCurrentAmps,
        turnStatorCurrentAmps,
        turnTorqueCurrentAmps);
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon, encoder));

    // Register signals for refresh
    PhoenixUtil.registerSignals(
        true,
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrentAmps,
        driveStatorCurrentAmps,
        driveTorqueCurrentAmps,
        turnPosition,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnSupplyCurrentAmps,
        turnStatorCurrentAmps,
        turnTorqueCurrentAmps);
  }

  @Override
  public void updateInputs(ModuleIO.ModuleIOInputs inputs) {
    // Update drive inputs
    inputs.driveConnected =
        BaseStatusSignal.isAllGood(
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveSupplyCurrentAmps,
            driveStatorCurrentAmps,
            driveTorqueCurrentAmps);
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrentAmps.getValueAsDouble();
    inputs.driveStatorCurrentAmps = driveStatorCurrentAmps.getValueAsDouble();
    inputs.driveTorqueCurrentAmps = driveTorqueCurrentAmps.getValueAsDouble();

    // Update turn inputs
    inputs.turnConnected =
        BaseStatusSignal.isAllGood(
            turnPosition,
            turnVelocity,
            turnAppliedVolts,
            turnSupplyCurrentAmps,
            turnStatorCurrentAmps,
            turnTorqueCurrentAmps);
    inputs.turnEncoderConnected = BaseStatusSignal.isAllGood(turnAbsolutePosition);
    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble()).minus(encoderOffset);
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnSupplyCurrentAmps = turnSupplyCurrentAmps.getValueAsDouble();
    inputs.turnStatorCurrentAmps = turnSupplyCurrentAmps.getValueAsDouble();
    inputs.turnTorqueCurrentAmps = turnTorqueCurrentAmps.getValueAsDouble();

    // Update odometry inputs
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble(Units::rotationsToRadians).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnTalon.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec, double feedforward) {
    driveTalon.setControl(
        velocityTorqueCurrentRequest
            .withVelocity(Units.radiansToRotations(velocityRadPerSec))
            .withFeedForward(feedforward));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(positionTorqueCurrentRequest.withPosition(rotation.getRotations()));
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveConfig.Slot0.kP = kP;
    driveConfig.Slot0.kI = kI;
    driveConfig.Slot0.kD = kD;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnConfig.Slot0.kP = kP;
    turnConfig.Slot0.kI = kI;
    turnConfig.Slot0.kD = kD;
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    brakeModeExecutor.execute(
        () -> {
          synchronized (driveConfig) {
            driveConfig.MotorOutput.NeutralMode =
                enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
          }
        });
    brakeModeExecutor.execute(
        () -> {
          synchronized (turnConfig) {
            turnConfig.MotorOutput.NeutralMode =
                enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));
          }
        });
  }
}
