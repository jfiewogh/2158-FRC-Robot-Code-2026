package org.austincans.frc2158.subsystems.hood;

import org.austincans.frc2158.math.Angle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodIOSim implements HoodIO{
    private final DCMotor motorBase = DCMotor.getKrakenX60(1);
    private DCMotorSim motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(
              motorBase, HoodConstants.rotationalInertiaMetersSq, HoodConstants.gearing),
          motorBase);
    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodAngleDeg = getAngle().toDegrees(0);
        inputs.temperatureCelsius = 0;
        inputs.motorPositionRotations = getAngle().toRotations();
        inputs.velocityRotationsPerSec = motor.getAngularVelocityRadPerSec();
    }

    @Override
    public void setAngle(Angle angle) {
        motor.setAngle(Units.rotationsToRadians(angle.toRotations()));
    }

    @Override
    public void setAngleRotation2d(Rotation2d angle) {
        motor.setAngle(angle.getRadians());
    }

    @Override
    public Angle getAngle() {
        return Angle.fromRotations(motor.getAngularPositionRotations());
    }

    @Override
    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromRadians(motor.getAngularPositionRad());
    }

    @Override
    public boolean atAngle(Angle setpoint) {
        return setpoint.toRadians() == motor.getAngularPositionRad();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setInputVoltage(voltage);
    }
    
}
