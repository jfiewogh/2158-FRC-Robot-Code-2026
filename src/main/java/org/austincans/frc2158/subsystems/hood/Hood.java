package org.austincans.frc2158.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Hood {
  void setAngle(Rotation2d angle);
  Rotation2d getAngle();
  boolean atAngle();
}
