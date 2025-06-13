// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.subsystems.vision;

import static org.austincans.frc2158.subsystems.vision.VisionConstants.angularStdDevBaseline;
import static org.austincans.frc2158.subsystems.vision.VisionConstants.angularStdDevMegatag2Factor;
import static org.austincans.frc2158.subsystems.vision.VisionConstants.cameraStdDevFactors;
import static org.austincans.frc2158.subsystems.vision.VisionConstants.linearStdDevBaseline;
import static org.austincans.frc2158.subsystems.vision.VisionConstants.linearStdDevMegatag2Factor;
import static org.austincans.frc2158.subsystems.vision.VisionConstants.maxAmbiguity;
import static org.austincans.frc2158.subsystems.vision.VisionConstants.maxZError;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.List;
import org.austincans.frc2158.FieldConstants.AprilTagLayoutType;
import org.austincans.frc2158.subsystems.drive.Drive;
import org.austincans.frc2158.subsystems.vision.VisionIO.PoseObservationType;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  public Vision(
      Drive drive,
      VisionIOPhotonVisionSim visionIOPhotonVisionSim,
      VisionIOPhotonVisionSim visionIOPhotonVisionSim2) {
    this.io = new VisionIO[] {visionIOPhotonVisionSim, visionIOPhotonVisionSim2};
    this.consumer = (VisionConsumer) drive;
    this.inputs = new VisionIOInputsAutoLogged[1];
    inputs[0] = new VisionIOInputsAutoLogged();
    inputs[1] = new VisionIOInputsAutoLogged();
    this.disconnectedAlerts = new Alert[1];
    disconnectedAlerts[0] = new Alert("Vision camera 0 is disconnected.", AlertType.kWarning);
    disconnectedAlerts[1] = new Alert("Vision camera 1 is disconnected.", AlertType.kWarning);
  }

  public Vision(
      Drive drive, VisionIOLimelight visionIOLimelight, VisionIOPhotonVision visionIOPhotonVision) {
    this.io = new VisionIO[] {visionIOLimelight, visionIOPhotonVision};
    this.consumer = (VisionConsumer) drive;
    this.inputs = new VisionIOInputsAutoLogged[1];
    inputs[0] = new VisionIOInputsAutoLogged();
    inputs[1] = new VisionIOInputsAutoLogged();
    this.disconnectedAlerts = new Alert[1];
    disconnectedAlerts[0] = new Alert("Vision camera 0 is disconnected.", AlertType.kWarning);
    disconnectedAlerts[1] = new Alert("Vision camera 1 is disconnected.", AlertType.kWarning);
  }

  public Vision(Drive drive, VisionIO visionIO, VisionIO visionIO2) {
    this.io = new VisionIO[] {visionIO, visionIO2};
    this.consumer = (VisionConsumer) drive;
    this.inputs = new VisionIOInputsAutoLogged[1];
    inputs[0] = new VisionIOInputsAutoLogged();
    inputs[1] = new VisionIOInputsAutoLogged();
    this.disconnectedAlerts = new Alert[1];
    disconnectedAlerts[0] = new Alert("Vision camera 0 is disconnected.", AlertType.kWarning);
    disconnectedAlerts[1] = new Alert("Vision camera 1 is disconnected.", AlertType.kWarning);
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX()
                    > AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY()
                    > AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
