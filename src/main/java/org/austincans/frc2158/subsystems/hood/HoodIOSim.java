package org.austincans.frc2158.subsystems.hood;

import org.austincans.frc2158.math.Angle;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class HoodIOSim implements HoodIO{
    DCMotorSim hoodMotorSim = new DCMotorSim(null, null, null);
    PIDController pid = new PIDController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD);

    @Override
    public Angle getAngle() {
        return Angle.fromRadians(Units.rotationsToRadians(hoodMotorSim.getAngularPositionRotations()));
    }

    public void setHoodAngle(Angle setPointDegrees){
        hoodMotorSim.setInput(pid.calculate(hoodMotorSim.getAngularPositionRad(),Units.degreesToRotations(setPointDegrees.toDegrees())));
    }

    @Override
    public void setAngleRotation2d(Rotation2d angle) {
        hoodMotorSim.setInput(pid.calculate(hoodMotorSim.getAngularPositionRad(),Units.degreesToRotations(angle.getDegrees())));
    }

    @Override
    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromRadians(Units.rotationsToRadians(hoodMotorSim.getAngularPositionRotations()));
    }

    @Override
    public boolean atAngle(Angle setPoint) {
        if (getAngle() == setPoint){
            return true;
        }
        else{
            return false;
        }
    }

    public void updateInputs(hoodIOInputs inputs){
        inputs.hoodAngleDegrees = getAngle();
        inputs.temperature = 0;
        inputs.motorPosition = hoodMotorSim.getAngularPositionRotations();
    }
    
}
