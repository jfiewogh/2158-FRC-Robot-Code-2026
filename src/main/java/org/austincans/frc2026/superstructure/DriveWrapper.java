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

package org.austincans.frc2026.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.austincans.frc2026.RobotState;
import org.austincans.frc2026.subsystems.drive.Drive;
import org.austincans.frc2026.subsystems.drive.DriveConstants;
import org.austincans.lib.superstructure.SubsystemWrapper;
import org.austincans.lib.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

// TODO: in the future, update this to use RobotState methods instead of getting them from drive
public class DriveWrapper extends SubsystemWrapper<Drive> {
  public static final double deadband = 0.1;
  private static final double ffStartDelay = 2.0; // Secs
  private static final double ffRampRate = 0.1; // Volts/Sec
  private static final double wheelRadiusMaxVelocity = 0.25; // Rad/Sec
  private static final double wheelRadiusRampRate = 0.05; // Rad/Sec^2

  @SuppressWarnings("unused")
  private static final LoggedTunableNumber driveKP =
      new LoggedTunableNumber("DriveCommands/kP", 6.0);

  @SuppressWarnings("unused")
  private static final LoggedTunableNumber driveKD =
      new LoggedTunableNumber("DriveCommands/kD", 0.05);

  private static final LoggedTunableNumber angularKP =
      new LoggedTunableNumber("DriveCommands/kP", 6.0);
  private static final LoggedTunableNumber angularKD =
      new LoggedTunableNumber("DriveCommands/kD", 0.05);

  private static final double ANGLE_MAX_VELOCITY = Math.PI;
  private static final double ANGLE_MAX_ACCELERATION = Math.PI * 2;

  // Create PID controller
  private final ProfiledPIDController angleController =
      new ProfiledPIDController(
          angularKP.getAsDouble(),
          0.0,
          angularKD.getAsDouble(),
          new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));

  public enum TurnMode {
    TWO_AXIS,
    ONE_AXIS
  }

  private TurnMode turnMode = TurnMode.ONE_AXIS;

  private Rotation2d lastFieldRelRotation = Rotation2d.kZero;

  public DriveWrapper(Optional<Drive> subsystem) {
    super(subsystem);

    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), deadband);
    Rotation2d linearDirection = new Rotation2d(x, y);

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  private void joystickDrive(
      DoubleSupplier xSupplier, // forward/backward
      DoubleSupplier ySupplier, // left/right
      DoubleSupplier omegaXSupplier, // forward/backward
      DoubleSupplier omegaYSupplier) {

    // Get linear velocity
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Apply rotation deadband
    double omega = MathUtil.applyDeadband(omegaYSupplier.getAsDouble(), deadband);

    // Square rotation value for more precise control
    omega = Math.copySign(omega * omega, omega);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    // Logger.recordOutput("flipped", isFlipped);

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * s.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * s.getMaxLinearSpeedMetersPerSec(),
            omega * s.getMaxAngularSpeedRadPerSec());

    lastFieldRelRotation = s.getRotation();

    s.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, isFlipped ? s.getRotation().plus(Rotation2d.k180deg) : s.getRotation()));
  }

  /**
   * Field or robot relative drive command using two joysticks (controlling linear and angular
   * velocities).
   */
  public Command joystickDriveCommand(
      DoubleSupplier xSupplier, // forward/backward
      DoubleSupplier ySupplier, // left/right
      DoubleSupplier omegaXSupplier, // forward/backward
      DoubleSupplier omegaYSupplier // left/right
      ) {
    if (!isPresent) {
      return Commands.none();
    }
    return Commands.run(
        () -> joystickDrive(xSupplier, ySupplier, omegaXSupplier, omegaYSupplier), s);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  private void joystickDriveAtAngle(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaXSupplier,
      DoubleSupplier omegaYSupplier) {

    Rotation2d rotation =
        Rotation2d.fromRadians(
            // horizontal / vertical (x on joystick is horiz, y on joystick is vert)
            Math.atan2(omegaYSupplier.getAsDouble(), omegaXSupplier.getAsDouble()));

    // Get linear velocity
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    // Calculate angular speed
    Rotation2d desiredRotation;
    if (Math.abs(omegaXSupplier.getAsDouble()) > 0.1
        && Math.abs(omegaYSupplier.getAsDouble()) > 0.1) {
      desiredRotation = isFlipped ? rotation.plus(Rotation2d.k180deg) : rotation;
      lastFieldRelRotation = desiredRotation;
    } else {
      desiredRotation = lastFieldRelRotation;
    }

    LoggedTunableNumber.ifChanged(
        angularKD.hashCode(), () -> angleController.setD(angularKD.getAsDouble()), angularKD);
    LoggedTunableNumber.ifChanged(
        angularKP.hashCode(), () -> angleController.setP(angularKP.getAsDouble()), angularKP);

    double omega =
        angleController.calculate(s.getRotation().getRadians(), desiredRotation.getRadians());

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * s.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * s.getMaxLinearSpeedMetersPerSec(),
            omega * s.getMaxAngularSpeedRadPerSec());
    s.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, isFlipped ? s.getRotation().plus(Rotation2d.k180deg) : s.getRotation()));
  }

  public Command joystickDriveAtAngleCommand(
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationXSupplier,
      DoubleSupplier rotationYSupplier) {
    if (!isPresent) {
      return Commands.none();
    }
    return Commands.run(
            () ->
                joystickDriveAtAngle(
                    translationXSupplier,
                    translationYSupplier,
                    rotationXSupplier,
                    rotationYSupplier),
            s)
        // Reset PID controller when command starts
        .beforeStarting(
            () -> {
              angleController.reset(s.getRotation().getRadians());
              lastFieldRelRotation = s.getRotation();
            });
  }

  public Command driveCommand(
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationXSupplier,
      DoubleSupplier rotationYSupplier) {
    if (!isPresent) {
      return Commands.none();
    }
    return Commands.run(
        () -> {
          if (turnMode == TurnMode.TWO_AXIS) {
            joystickDriveAtAngle(
                translationXSupplier, translationYSupplier, rotationXSupplier, rotationYSupplier);
          } else {
            joystickDrive(
                translationXSupplier, translationYSupplier, rotationXSupplier, rotationYSupplier);
          }
        },
        s);
  }

  /** No requirements needed */
  public Command setTurnModeCommand() {
    return Commands.runOnce(
        () -> {
          if (turnMode == TurnMode.ONE_AXIS) {
            turnMode = TurnMode.TWO_AXIS;
          } else {
            turnMode = TurnMode.ONE_AXIS;
          }
        });
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public Command feedforwardCharacterization() {
    if (!isPresent) {
      return Commands.none();
    }
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  s.runCharacterization(0.0);
                },
                s)
            .withTimeout(ffStartDelay),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * ffRampRate;
                  s.runCharacterization(voltage);
                  velocitySamples.add(s.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                s)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public Command wheelRadiusCharacterization() {
    if (!isPresent) {
      return Commands.none();
    }
    SlewRateLimiter limiter = new SlewRateLimiter(wheelRadiusRampRate);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(() -> limiter.reset(0.0)),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(wheelRadiusMaxVelocity);
                  s.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                s)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = s.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = RobotState.getInstance().getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = RobotState.getInstance().getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;

                      double[] positions = s.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                      Logger.recordOutput("Drive/WheelDelta", wheelDelta);
                      Logger.recordOutput("Drive/WheelRadius", wheelRadius);
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = s.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.00000000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return isPresent ? s.sysIdQuasistatic(direction) : Commands.none();
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return isPresent ? s.sysIdDynamic(direction) : Commands.none();
  }

  /**
   * Returns a command to run a dynamic test in the specified direction and with counterClockwise
   * Rotation.
   */
  public Command sysIdDynamicAngular(SysIdRoutine.Direction direction) {
    return isPresent ? s.sysIdDynamicAngular(direction) : Commands.none();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistaticAngular(SysIdRoutine.Direction direction) {
    return isPresent ? s.sysIdQuasistaticAngular(direction) : Commands.none();
  }

  public Command stopWithXCommand() {
    return isPresent ? Commands.runOnce(s::stopWithX, s) : Commands.none();
  }

  public Command zeroPoseCommand() {
    if (!isPresent) {
      return Commands.none();
    }
    return Commands.runOnce(
        () ->
            s.setPose(
                new Pose2d(
                    s.getPose().getTranslation(),
                    DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red
                        ? Rotation2d.k180deg
                        : Rotation2d.kZero)),
        s);
  }
}
