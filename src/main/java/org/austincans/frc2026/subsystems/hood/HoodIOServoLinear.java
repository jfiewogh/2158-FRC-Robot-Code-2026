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

package org.austincans.frc2026.subsystems.hood;

import com.revrobotics.servohub.ServoChannel.ChannelId;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import org.austincans.frc2026.controllers.ServoHubController;
import org.austincans.frc2026.math.Angle;

public class HoodIOServoLinear implements HoodIO {
  ServoHubController servoHub = new ServoHubController();
  // LinearActuatorController linearActuator;
  InterpolatingDoubleTreeMap angles = new InterpolatingDoubleTreeMap();
  Angle currentAngle = Angle.fromDegrees(0);

  public HoodIOServoLinear() {
    // this.linearActuator = new LinearActuatorController(HoodConstants.linearActuatorID);
    // angles.put(0.0, 1000.0); // (Angles, Position)  Angles [0, 90], Position [0, 1]
    // angles.put(90.0, 2000.0);

    // if (HoodConstants.potentiometer == true);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodAngleDeg = currentAngle.getRotation2d().getDegrees();
  }

  @Override
  public Angle getAngle() {
    return currentAngle;
  }

  @Override
  public void setAngle(Angle setpoint) {
    servoHub.setPulse(ChannelId.kChannelId0, (int) setpoint.getRotation2d().getDegrees());
    // currentAngle = Angle.fromDegrees(setPoint.toDegrees());
  }

  @Override
  public void set(int pulseWidth) {
    servoHub.setPulse(ChannelId.kChannelId0, pulseWidth);
  }

  @Override
  public void setAngleRotation2d(Rotation2d angle) {
    servoHub.setPulse(ChannelId.kChannelId0, (angles.get(angle.getDegrees()).intValue()));
    currentAngle = Angle.fromDegrees(angle.getDegrees());
  }

  /**
   * Calculate hood angle from linear actuator length
   * Uses law of cosines
   */
  private final double a = 0.2;
  private final double c = 0.2;
  private final double aSq = Math.pow(a, 2);
  private final double cSq = Math.pow(c, 2);
  public Rotation2d getAngleFromLength(double b) {
    double angleRad = Math.acos((aSq + Math.pow(b, 2) - cSq) / (2 * a * b));
    return Rotation2d.fromRadians(angleRad);
  }

  private static double map(double value, double iMin, double iMax, double oMin, double oMax) {
    return oMin + (oMax - oMin) * ((value - iMin) / (iMax - iMin));
  }

  // TODO: find actual min and maxvalues
  private final double minPulseWidth = 1000;
  private final double maxPulseWidth = 2000;
  private final double minLength = 0;
  private final double maxLength = 0.2;
  public double getLengthFromPulseWidth(double pulseWidth) {
    return map(pulseWidth, minPulseWidth, maxPulseWidth, minLength, maxLength);
  }

  public Rotation2d getAngleFromPulseWidth(double pulseWidth) {
    return getAngleFromLength(getLengthFromPulseWidth(pulseWidth));
  }

  public double getPulseWidthFromLength(double length) {
    return map(length, minLength, maxLength, minPulseWidth, maxPulseWidth);
  }

  public double getLengthFromAngle(Rotation2d angle) {
    // TODO: finish
    // c^2 = b^2 + a^2 - 2ab * cosC
    // return Math.sqrt(cSq + aSq - 2 * a * c * angle.getCos());
  }

  @Override
  public Rotation2d getAngleRotation2d() {
    return Rotation2d.fromDegrees(currentAngle.toDegrees());
  }

  @Override
  public boolean atAngle(Angle setpoint) {
    return currentAngle.toDegrees() == setpoint.toDegrees();
  }

  @Override
  public void setVoltage(double voltage) {}
}
