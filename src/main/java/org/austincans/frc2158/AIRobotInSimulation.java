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
// Updated on Thursday November 13, 2025 at 14:16:40 CST

package org.austincans.frc2158;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import java.util.function.Supplier;
import org.austincans.frc2158.subsystems.drive.Drive;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.utils.FieldMirroringUtils;

public class AIRobotInSimulation extends SubsystemBase {

  public static final AIRobotInSimulation[] instances =
      new AIRobotInSimulation[1]; // you can create as many opponent robots as you needs

  /* If an opponent robot is not on the field, it is placed in a queening position for performance. */
  public static final Pose2d[] ROBOT_QUEENING_POSITIONS =
      new Pose2d[] {
        new Pose2d(-6, 0, new Rotation2d()),
        new Pose2d(-5, 0, new Rotation2d()),
        new Pose2d(-4, 0, new Rotation2d()),
        new Pose2d(-3, 0, new Rotation2d()),
        new Pose2d(-2, 0, new Rotation2d())
      };

  // PathPlanner configuration
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          55, // Robot mass in kg
          8, // Robot MOI
          new ModuleConfig(
              Units.inchesToMeters(2),
              3.5,
              1.2,
              DCMotor.getNeoVortex(1).withReduction(8.14),
              60,
              1), // Swerve module config
          0.6 // Track length and width
          );

  private final SwerveDriveSimulation driveSimulation;
  private final SelfControlledSwerveDriveSimulation selfCtrlDriveSimulation;
  private final Pose2d queeningPose;
  private final int id;

  /* The drivetrain configuration for the opponent robots in the maple-sim simulation. */
  private static final DriveTrainSimulationConfig DRIVETRAIN_CONFIG = Drive.mapleSimConfig;

  // PathPlanner PID settings
  private final PPHolonomicDriveController driveController =
      new PPHolonomicDriveController(new PIDConstants(5.0, 0.02), new PIDConstants(7.0, 0.05));

  public AIRobotInSimulation(int id) {
    this.id = id;
    this.queeningPose = ROBOT_QUEENING_POSITIONS[id];

    this.driveSimulation = new SwerveDriveSimulation(DRIVETRAIN_CONFIG, queeningPose);
    this.selfCtrlDriveSimulation = new SelfControlledSwerveDriveSimulation(this.driveSimulation);
    SimulatedArena.getInstance()
        .addDriveTrainSimulation(selfCtrlDriveSimulation.getDriveTrainSimulation());
  }

  public void buildBehaviorChooser(
      PathPlannerPath segment0,
      Command toRunAtEndOfSegment0,
      PathPlannerPath segment1,
      Command toRunAtEndOfSegment1,
      XboxController joystick) {
    SendableChooser<Command> behaviorChooser = new SendableChooser<>();
    final Supplier<Command> disable =
        () ->
            Commands.runOnce(
                    () -> selfCtrlDriveSimulation.setSimulationWorldPose(queeningPose), this)
                .andThen(
                    Commands.runOnce(
                        () ->
                            selfCtrlDriveSimulation.runChassisSpeeds(
                                new ChassisSpeeds(), new Translation2d(), false, false)))
                .ignoringDisable(true);

    // Option to disable the robot
    behaviorChooser.setDefaultOption("Disable", disable.get());

    // Option to auto-cycle the robot
    behaviorChooser.addOption(
        "Auto Cycle",
        getAutoCycleCommand(segment0, toRunAtEndOfSegment0, segment1, toRunAtEndOfSegment1));

    // Option to manually control the robot with a joystick
    behaviorChooser.addOption("Joystick Drive", joystickDrive(joystick));

    // Schedule the command when another behavior is selected
    behaviorChooser.onChange((Command::schedule));

    // Schedule the selected command when teleop starts
    RobotModeTriggers.teleop()
        .onTrue(Commands.runOnce(() -> behaviorChooser.getSelected().schedule()));

    // Disable the robot when the user robot is disabled
    RobotModeTriggers.disabled().onTrue(disable.get());

    SmartDashboard.putData("AIRobotBehaviors/Opponent Robot " + id + " Behavior", behaviorChooser);
  }

  public static final Pose2d[] ROBOTS_STARTING_POSITIONS =
      new Pose2d[] {
        new Pose2d(15, 6, Rotation2d.fromDegrees(180)),
        new Pose2d(15, 4, Rotation2d.fromDegrees(180)),
        new Pose2d(15, 2, Rotation2d.fromDegrees(180)),
        new Pose2d(1.6, 6, new Rotation2d()),
        new Pose2d(1.6, 4, new Rotation2d())
      };

  /** Joystick drive command for opponent robots */
  private Command joystickDrive(XboxController joystick) {
    // Obtain chassis speeds from joystick input
    final Supplier<ChassisSpeeds> joystickSpeeds =
        () ->
            new ChassisSpeeds(
                -joystick.getLeftY()
                    * selfCtrlDriveSimulation.maxLinearVelocity().in(MetersPerSecond),
                -joystick.getLeftX()
                    * selfCtrlDriveSimulation.maxLinearVelocity().in(MetersPerSecond),
                -joystick.getRightX()
                    * selfCtrlDriveSimulation.maxAngularVelocity().in(RadiansPerSecond));

    // Obtain driverstation facing for opponent driver station
    @SuppressWarnings("unused")
    final Supplier<Rotation2d> opponentDriverStationFacing =
        () ->
            FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
                .plus(Rotation2d.fromDegrees(180));

    return Commands.run(
            () -> {
              // Calculate field-centric speed from driverstation-centric speed
              final ChassisSpeeds fieldCentricSpeeds =
                  ChassisSpeeds.fromRobotRelativeSpeeds(
                      joystickSpeeds.get(),
                      FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
                          .plus(Rotation2d.fromDegrees(180)));
              // Run the field-centric speed
              selfCtrlDriveSimulation.runChassisSpeeds(
                  fieldCentricSpeeds, new Translation2d(), true, true);
            },
            this)
        // Before the command starts, reset the robot to a position inside the field
        .beforeStarting(
            () ->
                selfCtrlDriveSimulation.setSimulationWorldPose(
                    FieldMirroringUtils.toCurrentAlliancePose(ROBOTS_STARTING_POSITIONS[id - 1])));
  }

  /** Follow path command for opponent robots */
  private Command opponentRobotFollowPath(PathPlannerPath path) {
    return new FollowPathCommand(
        path, // Specify the path
        // Provide actual robot pose in simulation, bypassing odometry error
        selfCtrlDriveSimulation::getActualPoseInSimulationWorld,
        // Provide actual robot speed in simulation, bypassing encoder measurement error
        selfCtrlDriveSimulation::getActualSpeedsRobotRelative,
        // Chassis speeds output
        (speeds, feedforwards) ->
            selfCtrlDriveSimulation.runChassisSpeeds(speeds, new Translation2d(), false, false),
        driveController, // Specify PID controller
        PP_CONFIG, // Specify robot configuration
        // Flip path based on alliance side
        () ->
            DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                .equals(DriverStation.Alliance.Red),
        this // AIRobotInSimulation is a subsystem; this command should use it as a requirement
        );
  }

  /** Get the command to auto-cycle the robot relatively */
  private Command getAutoCycleCommand(
      PathPlannerPath segment0,
      Command toRunAtEndOfSegment0,
      PathPlannerPath segment1,
      Command toRunAtEndOfSegment1) {
    final SequentialCommandGroup cycle = new SequentialCommandGroup();
    final Pose2d startingPose =
        new Pose2d(
            segment0.getStartingDifferentialPose().getTranslation(),
            segment0.getIdealStartingState().rotation());

    cycle.addCommands(
        opponentRobotFollowPath(segment0).andThen(toRunAtEndOfSegment0).withTimeout(10));
    cycle.addCommands(
        opponentRobotFollowPath(segment1).andThen(toRunAtEndOfSegment1).withTimeout(10));

    return cycle
        .repeatedly()
        .beforeStarting(
            Commands.runOnce(
                () ->
                    selfCtrlDriveSimulation.setSimulationWorldPose(
                        FieldMirroringUtils.toCurrentAlliancePose(startingPose))));
  }

  public static void startOpponentRobotSimulations() {
    try {
      instances[0] = new AIRobotInSimulation(0);
      instances[0].buildBehaviorChooser(
          PathPlannerPath.fromPathFile("opponent robot cycle path 0"),
          Commands.none(),
          PathPlannerPath.fromPathFile("opponent robot cycle path 0 backwards"),
          Commands.none(),
          new XboxController(2));

      instances[1] = new AIRobotInSimulation(1);
      instances[1].buildBehaviorChooser(
          PathPlannerPath.fromPathFile("opponent robot cycle path 1"),
          Commands.none(),
          PathPlannerPath.fromPathFile("opponent robot cycle path 1 backwards"),
          Commands.none(),
          new XboxController(3));

    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load opponent robot simulation paths, error: " + e.getMessage(), false);
    }
  }
}
