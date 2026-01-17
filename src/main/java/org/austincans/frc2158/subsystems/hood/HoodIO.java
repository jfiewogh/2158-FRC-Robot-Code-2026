package org.austincans.frc2158.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;

public interface HoodIO {
    void setAngle();

    Rotation2d getAngle();

    boolean atAngle();
}
