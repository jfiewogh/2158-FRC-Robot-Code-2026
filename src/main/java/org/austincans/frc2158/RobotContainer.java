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
// Updated on Sunday February 15, 2026 at 11:15:12 CST

package org.austincans.frc2158;

import static org.austincans.frc2158.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.austincans.frc2158.commands.DriveCommands;
import org.austincans.frc2158.commands.LimelightHelpersCommands;
import org.austincans.frc2158.commands.TurretTracking;
import org.austincans.frc2158.generated.TunerConstants;
import org.austincans.frc2158.subsystems.drive.*;
import org.austincans.frc2158.subsystems.flywheel.Flywheel;
import org.austincans.frc2158.subsystems.flywheel.FlywheelIOSim3D;
import org.austincans.frc2158.subsystems.hood.Hood;
import org.austincans.frc2158.subsystems.hood.HoodIOTest;
import org.austincans.frc2158.subsystems.intake.Intake;
import org.austincans.frc2158.subsystems.intake.IntakeIOSim3D;
import org.austincans.frc2158.subsystems.turret.Turret;
import org.austincans.frc2158.subsystems.turret.TurretIOSim;
import org.austincans.frc2158.subsystems.vision.*;
import org.ironmaple.simulation.IntakeSimulation3D;
import org.ironmaple.simulation.SimulatedArena3D;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation3D;
import org.ironmaple.simulation.physics.threading.PhysicsThreadConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt3D;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  @SuppressWarnings("unused")
  private final Vision vision;

  private ObjectDetection objectDetection;

  private SwerveDriveSimulation3D driveSimulation = null;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private IntakeSimulation3D intakeSimulation;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    DoubleSupplier leftY = () -> -controller.getLeftY();
    DoubleSupplier leftX = () -> -controller.getLeftX();
    DoubleSupplier rightY = () -> -controller.getRightX();
    DoubleSupplier rightX = () -> -controller.getRightY();

    switch (Constants.currentMode) {
      case REAL -> {
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new DriveIOReal(),
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight),
                (pose) -> {});
        this.vision =
            new Vision(
                drive,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
      }
      case SIM -> {
        // Sim robot, instantiate physics sim IO implementations

        // Initialize Rebuilt Arena
        boolean isThreaded = false;

        if (isThreaded) {
          new Arena2026Rebuilt3D(PhysicsThreadConfig.enabled(100));
        } else {
          new Arena2026Rebuilt3D();
        }

        driveSimulation = new SwerveDriveSimulation3D(Drive.mapleSimConfig, Pose2d.kZero);
        driveSimulation.registerWithArena(
            SimulatedArena3D.getInstance(), new Pose2d(14, 3, new Rotation2d()));

        drive =
            new Drive(
                new DriveIOSim3D(driveSimulation),
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

        double intakeWidth = Constants.RobotConstants.kBumperWidthMeters;
        double intakeHeight = 0.1;
        double intakeDepth = 0.1;
        intakeSimulation =
            new IntakeSimulation3D(
                "Fuel",
                driveSimulation,
                SimulatedArena3D.getInstance()
                    .getPhysicsEngine()
                    .createBoxShape(
                        new Translation3d(intakeDepth / 2, intakeWidth / 2, intakeHeight / 2)),
                new Transform3d(
                    DriveConstants.trackWidthX / 2 + intakeDepth / 2,
                    0,
                    intakeHeight / 2,
                    Rotation3d.kZero),
                35,
                SimulatedArena3D.getInstance());

        objectDetection =
            new ObjectDetection(
                // new ObjectDetectionIOSimInRange(
                // "Fuel", 1, driveSimulation::getSimulatedDriveTrainPose3dGroundRelative));
                new ODIOSimInView(
                    "Fuel",
                    LimelightConfig.LIMELIGHT_3G,
                    4,
                    driveSimulation::getSimulatedDriveTrainPose3dGroundRelative,
                    () -> new Pose3d()));

        KeyboardInput keyboardInput = new KeyboardInput();

        leftX = keyboardInput::leftX;
        leftY = keyboardInput::leftY;
        rightX = keyboardInput::rightX;
        rightY = keyboardInput::rightY;

        if (intakeSimulation != null) {
          Hood hood = new Hood(new HoodIOTest());
          Turret turret = new Turret(new TurretIOSim(hood::getAngle));
          Intake intake = new Intake(new IntakeIOSim3D(intakeSimulation));

          Flywheel flywheel =
              new Flywheel(
                  new FlywheelIOSim3D(
                      driveSimulation,
                      intakeSimulation,
                      hood::getAngle,
                      () -> Rotation2d.fromDegrees(turret.getTurretAngle())));

          Command intakeCommand =
              Commands.runEnd(
                  () -> intake.setRelativeVelocity(1), () -> intake.setRelativeVelocity(0));
          Command shootCommand =
              new TurretTracking(
                  turret,
                  flywheel,
                  hood,
                  RobotState.getInstance()::getEstimatedPose,
                  RobotState.getInstance()::getRobotVelocity);
        Command intakeCommand =
            Commands.runEnd(
                () -> intake.setRelativeVelocity(1), () -> intake.setRelativeVelocity(0));
        Command shootCommand = TurretCommands.shootToHub(
            flywheel, 
            turret, 
            hood, 
            false, 
            () -> intakeSimulation.getGamePiecesInIntake() > 0
        );

          // keyboardInput.z().whileTrue(shootCommand);
          // keyboardInput.v().whileTrue(intakeCommand);
          // keyboardInput
          //     .z()
          //     .whileTrue(LimelightHelpersCommands.alignHeadingCommand(drive, objectDetection));
        //   // keyboardInput
        //   //     .x()
        //   //     .whileTrue(PathCommands.followPathToPose(drive, new Pose2d(8, 4,
          //
        // Rotation2d.kZero)));
          // keyboardInput
          //     .c()
          //     .whileTrue(PathCommands.followPathToPose(drive, new Pose2d(14, 3,
          // Rotation2d.kZero)));

          NamedCommands.registerCommand("runIntake", intakeCommand);
          NamedCommands.registerCommand("shoot", shootCommand);
        NamedCommands.registerCommand(
            "shuttle", TurretCommands.shuttle(flywheel, turret, hood, false));
        } else {
          NamedCommands.registerCommand("runIntake", Commands.none());
          NamedCommands.registerCommand("shoot", Commands.none());
        }
      }
      default -> {
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new DriveIO() {},
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});
        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
      }
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption("Top Auto", AutoBuilder.buildAuto("Top Auto"));
    autoChooser.addOption("Mid Auto", AutoBuilder.buildAuto("Mid Auto"));
    autoChooser.addOption("Shuttling Auto", AutoBuilder.buildAuto("Shuttling Auto"));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, leftY, leftX, rightX, rightY));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Lock to 0° when A button is held
    // controller
    // .a()
    // .whileTrue(
    // DriveCommands.joystickDriveAtAngle(
    // drive,
    // () -> -controller.getLeftY(),
    // () -> -controller.getLeftX(),
    // () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro / odometry
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () ->
                drive.setPose(
                    driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
            // simulation
            : () ->
                drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
    controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(14, 3, new Rotation2d()));
    SimulatedArena3D.getInstance().resetFieldForAuto();
  }

  public void preloadSimulation() {
    if (intakeSimulation != null) {
      intakeSimulation.addGamePiecesToIntake(20);
    }
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena3D.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition",
        driveSimulation.getSimulatedDriveTrainPose3dGroundRelative());
    Logger.recordOutput(
        "FieldSimulation/Fuel",
        SimulatedArena3D.getInstance().getGamePiecePosesArrayByType("Fuel"));
    // Logger.recordOutput(
    // "FieldSimulation/Fuel2D",
    // SimulatedArena.getInstance().getGamePiecePosesArrayByType("Fuel"));
  }
}
