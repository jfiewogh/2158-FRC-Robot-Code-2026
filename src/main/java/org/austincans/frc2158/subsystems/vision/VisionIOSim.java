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
// Updated on Sunday February 01, 2026 at 15:05:51 CST

package org.austincans.frc2158.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers.LimelightResults;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers.LimelightTarget_Classifier;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers.LimelightTarget_Detector;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers.RawDetection;

public class VisionIOSim implements VisionIO {

  String limelightName = "a";

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    RawDetection[] detections = LimelightHelpers.getRawDetections(limelightName);
    for (RawDetection detection : detections) {
      double txnc = detection.txnc;
      Rotation2d xOffset = Rotation2d.fromDegrees(txnc);
    }

    LimelightResults limelightResults = LimelightHelpers.getLatestResults(limelightName);

    LimelightTarget_Classifier[] classifier = limelightResults.targets_Classifier;
    LimelightTarget_Classifier closestClassifierObject = classifier[0];

    LimelightTarget_Detector[] detector = limelightResults.targets_Detector;
    LimelightTarget_Detector closestTargetObject = detector[0];
  }
}
