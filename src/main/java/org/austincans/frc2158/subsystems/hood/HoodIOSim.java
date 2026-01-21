// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.austincans.frc2158.subsystems.hood;

import static edu.wpi.first.units.Units.Rotation;

import org.austincans.frc2158.math.Angle;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class HoodIOSim implements HoodIO{
    public HoodIOSim(){}

    Angle angle = Angle.fromDegrees(0);

    @Override
    public Angle getAngle() {
        return angle;
    }

    @Override
    public void setAngle(Angle setPointDegrees) {
        angle = setPointDegrees;
    }

    @Override
    public void setAngleRotation2d(Rotation2d setPoint) {
        angle = Angle.fromDegrees(setPoint.getDegrees());
    }

    @Override
    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromDegrees(angle.toDegrees());
    }

    @Override
    public boolean atAngle(Angle setPoint) {
        return true;
    }
}
