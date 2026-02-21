package org.austincans.frc2158.subsystems.hood;

import java.util.List;

import org.austincans.frc2158.math.Angle;
import org.austincans.frc2158.subsystems.selfcheck.SelfChecking;
import org.austincans.frc2158.subsystems.selfcheck.SubsystemFault;

import edu.wpi.first.math.geometry.Rotation2d;

public class HoodIOTest implements HoodIO {
    private Rotation2d currentAngle;

    @Override
    public void updateInputs(HoodIOInputs inputs) {
      inputs.hoodAngleDeg = currentAngle.getDegrees();
    }

    @Override
    public void setAngle(Angle angle) {
      currentAngle = angle.getRotation2d();
    }

    @Override
    public void setAngleRotation2d(Rotation2d angle) {
      currentAngle = angle;
    }

    @Override
    public Angle getAngle() {
      return Angle.fromRotation2d(currentAngle);
    }

    @Override
    public Rotation2d getAngleRotation2d() {
      return currentAngle;
    }

    public boolean atAngle(Angle setpoint) {
      return currentAngle == setpoint.getRotation2d();
    }
}
