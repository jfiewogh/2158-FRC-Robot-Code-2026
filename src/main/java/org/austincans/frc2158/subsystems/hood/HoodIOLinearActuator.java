package org.austincans.frc2158.subsystems.hood;

import java.util.Map;

import org.austincans.frc2158.controllers.LinearActuatorController;
import org.austincans.frc2158.math.Angle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class HoodIOLinearActuator implements HoodIO{
    HoodIOInputs inputs;
    LinearActuatorController linearActuator;
    InterpolatingDoubleTreeMap angles = new InterpolatingDoubleTreeMap();
    Angle currentAngle = Angle.fromDegrees(0);
    public HoodIOLinearActuator(){
        this.linearActuator = new LinearActuatorController(HoodConstants.linearActuatorID);
        angles.put(0.0,0.0); // Angles [0, 90], Position [0, 1]
    }

    @Override
    public Angle getAngle() {
        return currentAngle;
    }

    @Override
    public void setAngle(Angle setPointDegrees) {
        linearActuator.setSpeed(angles.get(setPointDegrees.toDegrees()));
        currentAngle = Angle.fromDegrees(setPointDegrees.toDegrees());
    }

    @Override
    public void setAngleRotation2d(Rotation2d angle) {
        linearActuator.setSpeed(angles.get(angle.getDegrees()));
        currentAngle = Angle.fromDegrees(angle.getDegrees());
    }

    @Override
    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromDegrees(currentAngle.toDegrees());
    }

    @Override
    public boolean atAngle(Angle setPoint) {
        return currentAngle.toDegrees() == setPoint.toDegrees();
    }

}
