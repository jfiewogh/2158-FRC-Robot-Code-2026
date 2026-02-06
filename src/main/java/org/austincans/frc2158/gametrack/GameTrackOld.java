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
// Updated on Thursday February 05, 2026 at 19:51:21 CST

package org.austincans.frc2158.gametrack;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.ArrayList;
import java.util.List;
import org.austincans.frc2158.subsystems.drive.Drive;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers.LimelightResults;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers.LimelightTarget_Detector;
import org.ironmaple.simulation.SimulatedArena;

public class GameTrackOld {
  // Create the constraints to use while pathfinding
  public static PathConstraints constraints =
      new PathConstraints(5.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  // TODO: methods in here rely solely on MapleSim, update or create a new version that works with
  // real robot

  // For sim
  public static Pose3d[] getCoralPoses() {
    return SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
  }

  public static Pose3d[] getAlgaePoses() {
    return SimulatedArena.getInstance().getGamePiecesArrayByType("algae");
  }

  // only works if there is at least one coral we know the position of
  public static Pose3d getClosestCoralPose(Pose2d drivePose) {
    Pose3d[] coralPoses = getCoralPoses();

    Pose3d closestCoralPose = coralPoses[0];

    for (int i = 1; i < coralPoses.length; i++) {
      Pose3d coralPose = coralPoses[i];
      if (drivePose.getTranslation().getDistance(coralPose.getTranslation().toTranslation2d())
          < drivePose
              .getTranslation()
              .getDistance(closestCoralPose.getTranslation().toTranslation2d())) {
        closestCoralPose = coralPose;
      }
    }

    return closestCoralPose;
  }

  private static final double offsetMeters = 1;

  // -180 to 180
  @SuppressWarnings("unused")
  private static double normalizeAngle(double angle) {
    angle = angle % (Math.PI * 2);
    if (angle < -Math.PI) {
      angle += Math.PI * 2;
    }
    if (angle >= -Math.PI) {
      angle -= Math.PI * 2;
    }
    return angle;
  }

  // may return null
  public static PathPlannerPath getPath(Translation2d start, Pose2d goal) {
    Pathfinding.setStartPosition(start);
    Pathfinding.setGoalPosition(goal.getTranslation());
    return Pathfinding.getCurrentPath(constraints, new GoalEndState(0, goal.getRotation()));
  }

  public static void setPath(Translation2d start, Translation2d goal) {
    Pathfinding.setStartPosition(start);
    Pathfinding.setGoalPosition(goal);
  }

  public static double getPathTime(
      PathPlannerPath path, ChassisSpeeds speeds, Rotation2d rotation, RobotConfig config) {
    return path.generateTrajectory(speeds, rotation, config).getTotalTimeSeconds();
  }

  // vertical if pitch is 90 deg
  private static boolean coralIsVertical(Pose3d coralPose) {
    return coralPose.getRotation().getY() == Math.PI / 2;
  }

  public static void updateObstacles(Pose2d drivePose) {
    List<Pair<Translation2d, Translation2d>> obstacles =
        new ArrayList<Pair<Translation2d, Translation2d>>();
    obstacles.addAll(updateCoralObstacles(getCoralPoses()));
    obstacles.addAll(updateAlgaeObstacles(getAlgaePoses()));
    Pathfinding.setDynamicObstacles(obstacles, drivePose.getTranslation());
  }

  // TODO: ignore scored coral (do this by tracking scored coral, this will also help with score
  // tracking)

  public static Command getPathCommandToIntakeCoral(Drive drive) {
    Pose3d coralPose = getClosestCoralPose(drive.getPose());
    double coralYawRad = coralPose.getRotation().getZ();

    Pose2d drivePose = drive.getPose();
    Rotation2d driveRotation = drivePose.getRotation();

    // obstacles
    updateObstacles(drivePose);

    // if vertical (pitch is 90)
    if (coralIsVertical(coralPose)) {

      System.out.println("vertical");

      double rad =
          Math.atan2(coralPose.getY() - drivePose.getY(), coralPose.getX() - drivePose.getX());

      Pose2d desiredPose =
          new Pose2d(
              new Translation2d(
                  coralPose.getX() - offsetMeters * Math.cos(rad),
                  coralPose.getY() - offsetMeters * Math.sin(rad)),
              Rotation2d.fromRadians(rad));
      setPath(drivePose.getTranslation(), desiredPose.getTranslation());

      return AutoBuilder.pathfindToPose(desiredPose, constraints, 0)
          .raceWith(new WaitUntilCommand(() -> Drive.joystickHasInput));
    }

    // if horizontal

    double desiredRad = coralYawRad - Math.PI / 2;
    Pose2d firstPose =
        new Pose2d(
            new Translation2d(
                coralPose.getX() - offsetMeters * Math.cos(desiredRad),
                coralPose.getY() - offsetMeters * Math.sin(desiredRad)),
            Rotation2d.fromRadians(desiredRad));
    PathPlannerPath firstPath = getPath(drivePose.getTranslation(), firstPose);

    desiredRad = coralYawRad + Math.PI / 2;
    Pose2d secondPose =
        new Pose2d(
            new Translation2d(
                coralPose.getX() - offsetMeters * Math.cos(desiredRad),
                coralPose.getY() - offsetMeters * Math.sin(desiredRad)),
            Rotation2d.fromRadians(desiredRad));
    PathPlannerPath secondPath = getPath(drivePose.getTranslation(), secondPose);

    System.out.println(firstPose.toString() + "\n" + secondPose.toString());

    if (firstPath == null) {
      if (secondPath == null) {
        return Commands.runOnce(() -> {});
      }
      System.out.println("Only second");
      return AutoBuilder.pathfindToPose(secondPose, constraints, 0);
    } else if (secondPath == null) {
      System.out.println("Only first");
      return AutoBuilder.pathfindToPose(firstPose, constraints, 0);
    }

    double firstTime =
        firstPath
            .generateTrajectory(drive.getChassisSpeeds(), driveRotation, Drive.PP_CONFIG)
            .getTotalTimeSeconds();
    double secondTime =
        secondPath
            .generateTrajectory(drive.getChassisSpeeds(), driveRotation, Drive.PP_CONFIG)
            .getTotalTimeSeconds();

    System.out.println(firstTime + ", " + secondTime);

    if (firstTime < secondTime) {
      System.out.println("First faster");
      return AutoBuilder.pathfindToPose(firstPose, constraints, 0);
    }
    System.out.println("Second faster");
    return AutoBuilder.pathfindToPose(secondPose, constraints, 0);
  }

  public Pair<Translation2d, Translation2d> convertLimeLightToTranslation(
      LimelightTarget_Detector object) {

    return null;
  }

  @SuppressWarnings("unused")
  public void addObstacle(Pair<Translation2d, Translation2d> obstacle) {
    LimelightResults results = LimelightHelpers.getLatestResults("");
    LimelightTarget_Detector detection = null;
    if (results.targets_Detector.length > 0) {
      detection = results.targets_Detector[0];
      String className = detection.className;
      double confidence = detection.confidence;
      double area = detection.ta;
    }
    // obstacles.add(obstacle);
  }

  private static final double coralHeightMeters = Units.inchesToMeters(11.875);
  private static final double coralDiameterMeters = Units.inchesToMeters(4);
  private static final double coralRadiusMeters = coralDiameterMeters / 2;

  private static final double algaeRadiusMeters = Units.inchesToMeters(8);

  public static List<Pair<Translation2d, Translation2d>> updateCoralObstacles(Pose3d[] coralPoses) {
    List<Pair<Translation2d, Translation2d>> obstacles =
        new ArrayList<Pair<Translation2d, Translation2d>>();
    for (int i = 0; i < coralPoses.length; i++) {
      Pose3d pose = coralPoses[i];
      // If vertical
      if (coralIsVertical(pose)) {
        obstacles.add(
            new Pair<Translation2d, Translation2d>(
                new Translation2d(pose.getX() - coralRadiusMeters, pose.getY() - coralRadiusMeters),
                new Translation2d(
                    pose.getX() + coralRadiusMeters, pose.getY() + coralRadiusMeters)));
      }
      // If horizontal
      else {
        obstacles.add(
            new Pair<Translation2d, Translation2d>(
                new Translation2d(pose.getX() - coralHeightMeters, pose.getY() - coralHeightMeters),
                new Translation2d(
                    pose.getX() + coralHeightMeters, pose.getY() + coralHeightMeters)));
      }
    }
    return obstacles;
  }

  public static List<Pair<Translation2d, Translation2d>> updateAlgaeObstacles(Pose3d[] algaePoses) {
    List<Pair<Translation2d, Translation2d>> obstacles =
        new ArrayList<Pair<Translation2d, Translation2d>>();
    for (int i = 0; i < algaePoses.length; i++) {
      Pose3d pose = algaePoses[i];
      obstacles.add(
          new Pair<Translation2d, Translation2d>(
              new Translation2d(pose.getX() - algaeRadiusMeters, pose.getY() - algaeRadiusMeters),
              new Translation2d(pose.getX() + algaeRadiusMeters, pose.getY() + algaeRadiusMeters)));
    }
    return obstacles;
  }
}
