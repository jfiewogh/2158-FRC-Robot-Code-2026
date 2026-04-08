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

package org.austincans.frc2026.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.frc2026.subsystems.turret.TurretIO.TurretIOOutputs;
import org.austincans.frc2026.subsystems.vision.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

public class Turret extends AdvancedSubsystem {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final TurretIOOutputs outputs = new TurretIOOutputs();

  // private final VisionIO vision;
  // private final VisionIOInputsAutoLogged visionInputs = new
  // VisionIOInputsAutoLogged();
  private final DigitalInput magneticLimit = new DigitalInput(0); // id is zero
  private boolean previousValue = false;

  public Turret(TurretIO io) { // , VisionIO turretViz) {
    this.io = io;
    io.registerSelfCheckHardware(this);
    registerCheckCommand(systemCheckCommand());
    io.setDesiredAngle(Rotation2d.fromDegrees(0));
    // this.vision = turretViz;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // vision.updateInputs(visionInputs);
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret/Magnetic Limit Switch", magneticLimit.get());
    // if (inputs.motorVelocityRotationsPerSec < 0) {
    // if (magneticLimit.get() == true && magneticLimit.get() != previousValue) {
    // io.resetAngle(Rotation2d.kZero);
    // previousValue = magneticLimit.get();
    // System.out.println("Zero'd turret");
    // }
    // }

    // previousValue = magneticLimit.get();
    Rotation2d ccwPositiveRobotRelYaw =
        Rotation2d.fromRadians(inputs.positionRad)
            .unaryMinus()
            .plus(TurretConstants.robotRelTurretRotationOffset);

    double gearToCameraMeters = 0.2665;
    double forwardMeters =
        -0.123
            + gearToCameraMeters
                * ccwPositiveRobotRelYaw.getCos(); // meters, forward from robot center
    // right positive
    // negative because sin of ccw positive is left positive
    double sideMeters =
        -gearToCameraMeters * ccwPositiveRobotRelYaw.getSin(); // meters, left of robot center

    double upMeters = 0.331; // meters, up from robot center
    double rollDeg = 0.0; // degrees
    double pitchDeg = 15.0; // degrees (e.g. camera tilted down as arm moves)
    double yawDeg = ccwPositiveRobotRelYaw.getDegrees();

    /*
     *
     */
    LimelightHelpers.setCameraPose_RobotSpace(
        "limelight-turret", forwardMeters, sideMeters, upMeters, rollDeg, pitchDeg, yawDeg);
    // Logger.processInputs("TurretVision", visionInputs);

    // Pose2d pose = LimelightHelpers.getBotPose2d("limelight-turret");
    // Logger.recordOutput("Turret field rel pose", pose);
    // io.resetAngle(pose.getRotation().plus(TurretConstants.robotRelTurretRotationOffset));
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  @Override
  public void simulationPeriodic() {}

  public void setTurretVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setTurretVelocity(double velocityRadPerSec) {
    io.setVelocity(velocityRadPerSec);
  }

  /**
   * Get the value of the absolute encoder
   *
   * @return Absolute turret angle in radians
   */
  public double getTurretRads() {
    return inputs.positionRad;
  }

  public Rotation2d getTurretRotation2d() {
    return Rotation2d.fromRadians(inputs.positionRad);
  }

  /**
   * Create a command to set the turret angle
   *
   * @param turretAngle Desired turret angle
   * @return Command to rotate turret to given angle
   */
  public Command setTurretAngleCommand(Rotation2d turretAngle) {
    return Commands.run(() -> io.setDesiredAngle(turretAngle), this);
  }

  public void pointTurretToPosition(Translation3d targetPos, double aimPointLeft) {
    Pose2d robotPose = new Pose2d(); // TODO insert robot pose
    Rotation2d jointToTargetAngle =
        targetPos.toTranslation2d().minus(robotPose.getTranslation()).getAngle();

    double distance = targetPos.toTranslation2d().getDistance(robotPose.getTranslation());

    double cosC =
        (Math.pow(distance, 2) + Math.pow(distance, 2) - Math.pow(aimPointLeft, 2))
            / (2 * distance * distance);
    double angleOffset = Math.copySign(Math.acos(cosC), aimPointLeft);

    setDesiredAngleFieldRelative(
        jointToTargetAngle.minus(new Rotation2d(angleOffset)), robotPose.getRotation());
  }

  public double getTurretVelocity() {
    return inputs.velocityRadPerSec;
  }

  public Command pointTurretToPositionCommand(Translation3d fieldPos, double aimPointLeft) {
    return Commands.run(() -> pointTurretToPosition(fieldPos, aimPointLeft), this);
  }

  /**
   * Create a command to move the turret to the resting position
   *
   * @return Command to move turret to resting position
   */
  public Command turretRestingCommand() {
    return Commands.run(() -> io.setDesiredAngle(Rotation2d.fromDegrees(0)), this);
    // .until(() -> magLimitSwitch.get());
  }

  /** Relative to turret's zero */
  public void setDesiredAngle(Rotation2d turretRelAngle) {
    outputs.angle = turretRelAngle.unaryMinus();
  }

  /** Relative to robot's zero */
  public void setDesiredAngleRobotRelative(Rotation2d robotRelativeAngle) {
    setDesiredAngle(robotRelativeAngle.plus(TurretConstants.robotRelTurretRotationOffset));
  }

  /**
   * Set the desired angle of the turret to a field relative angle
   * @param fieldRelativeAngle The field relative angle to set turret to
   * @param currentPose Current pose of the robot
   */
  public void setDesiredAngleFieldRelative(Rotation2d fieldRelativeAngle, Rotation2d driveFieldRelRotation) {
    // unary minus because turret angle is clockwise positive, 
    // while robot rotation is counterclockwise positive
    setDesiredAngleRobotRelative(fieldRelativeAngle.minus(driveFieldRelRotation));
  }

  /**
   * Create a command to hold the turret at a field relative angle
   *
   * @param fieldRelativeAngle The field relative angle to hold, in degrees
   * @return Command to hold turret at field relative angle
   */
  public Command fieldRelativeAngleCommand(
      Rotation2d fieldRelativeAngle, Rotation2d robotRotation) {
    return Commands.run(
        () -> setDesiredAngleFieldRelative(fieldRelativeAngle, robotRotation), this);
  }

  /**
   * Create a command to move the turret angle manually
   *
   * @param angleControl gets the button configured to command
   * @return A command to move the turret angle manually
   */
  public Command angleManualControl(DoubleSupplier angleControl) {
    return Commands.run(
        () ->
            io.setDesiredAngle(
                Rotation2d.fromRadians(getTurretRads() + angleControl.getAsDouble() * 1)),
        this);
  }

  public Command resetAngle() {
    return Commands.runOnce(
        () -> {
          io.resetAngle(Rotation2d.kZero);
        },
        this);
  }

  public Command runTurret(double velocityRadPerSec) {
    return Commands.run(() -> setTurretVelocity(velocityRadPerSec));
  }

  @Override
  public Command systemCheckCommand() {
    return Commands.sequence(
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.velocityRadPerSec < Units.degreesToRadians(5)) {
                    addSubsystemFault(
                        "TRRT",
                        "[System Check] Turret absolute encoder velocity measured too slow",
                        false);
                  }
                  io.setVelocity(0);
                },
                this),
            Commands.runOnce(() -> io.setDesiredAngle(Rotation2d.fromDegrees(-20.0)), this),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  if (getTurretRads() < Units.degreesToRadians(-25)
                      || getTurretRads() > Units.degreesToRadians(-15)) {
                    addSubsystemFault(
                        "TRRT", "[System Check] Turret did not reach desired angle", false);
                  }
                },
                this),
            Commands.runOnce(() -> io.setDesiredAngle(Rotation2d.fromDegrees(0.0)), this),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  if (Math.abs(getTurretRads()) > Units.degreesToRadians(5)) {
                    addSubsystemFault(
                        "TRRT", "[System Check] Turret did not reach desired angle", false);
                  }
                },
                this),
            Commands.runOnce(
                () -> {
                  io.setDesiredAngle(Rotation2d.fromDegrees(0));
                },
                this),
            Commands.waitSeconds(2))
        // Commands.runOnce(
        // () -> {
        // if (magLimitSwitch.get() != true) {
        // addSubsystemFault(
        // "TRRT", "[System Check] Turret did not reach it's zero position");
        // }
        // }))
        .andThen(Commands.runOnce(() -> io.setVelocity(0), this));
  }
}
