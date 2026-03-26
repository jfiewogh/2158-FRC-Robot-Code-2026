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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import org.austincans.frc2026.controllers.ServoHubController;
import org.austincans.frc2026.subsystems.selfcheck.HardwareFault;
import org.austincans.frc2026.subsystems.selfcheck.SelfChecking;

public class HoodIOServoLinear implements HoodIO {
  private final ServoHubController servoHub = new ServoHubController();
  private final ChannelId id = ChannelId.kChannelId0;

  // TODO: find actual min and maxvalues
  private final double minPulseWidth = 1000;
  private final double maxPulseWidth = 1500;
  private final double minActuatorLengthMeters = 0;
  private final double maxActuatorLengthMeters = 0.2;

  private final double pivotToActuatorMountMeters = 0.2;
  private final double pivotToActuatorEndMeters = 0.2;
  private final double pivotToActuatorMountSq = Math.pow(pivotToActuatorMountMeters, 2);
  private final double pivotToActuatorEndSq = Math.pow(pivotToActuatorEndMeters, 2);

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.pulseWidth = servoHub.getPulseWidth(id);
    inputs.hoodAngle = getAngleFromPulseWidth(inputs.pulseWidth);
    inputs.shotPitchAngle = Rotation2d.fromDegrees(90).minus(inputs.hoodAngle);
  }

  @Override
  public void setPulse(int pulseWidth) {
    servoHub.setPulse(id, pulseWidth);
  }

  @Override
  public void setHoodAngle(Rotation2d angle) {
    setPulse(HoodConstants.angleToPWM.get(angle.getDegrees()).intValue());
  }

  /** Calculate hood angle from actuator length using law of cosines */
  private Rotation2d getAngleFromLength(double lengthMeters) {
    double angleRad =
        Math.acos(
            (pivotToActuatorMountSq + pivotToActuatorEndSq - Math.pow(lengthMeters, 2))
                / (2 * pivotToActuatorMountMeters * pivotToActuatorEndMeters));
    return Rotation2d.fromRadians(angleRad);
  }

  private static double map(double value, double iMin, double iMax, double oMin, double oMax) {
    value = MathUtil.clamp(value, iMin, iMax);
    return oMin + (oMax - oMin) * ((value - iMin) / (iMax - iMin));
  }

  private double getLengthFromPulseWidth(int pulseWidth) {
    return map(
        pulseWidth, minPulseWidth, maxPulseWidth, minActuatorLengthMeters, maxActuatorLengthMeters);
  }

  private Rotation2d getAngleFromPulseWidth(int pulseWidth) {
    return getAngleFromLength(getLengthFromPulseWidth(pulseWidth));
  }

  private int getPulseWidthFromLength(double length) {
    return (int)
        map(length, minActuatorLengthMeters, maxActuatorLengthMeters, minPulseWidth, maxPulseWidth);
  }

  private double getLengthFromAngle(Rotation2d angle) {
    // c^2 = a^2 + b^2 - 2ab * cosC. We are solving for c.
    // c = sqrt(b^2 + a^2 - 2ab * cosC)
    return Math.sqrt(
        pivotToActuatorMountSq
            + pivotToActuatorEndSq
            - 2 * pivotToActuatorMountMeters * pivotToActuatorEndMeters * angle.getCos());
  }

  private int getPulseWidthFromAngle(Rotation2d angle) {
    return getPulseWidthFromLength(getLengthFromAngle(angle));
  }

  @Override
  public List<HardwareFault> checkForFaults() {
    return List.of();
  }

  @Override
  public List<SelfChecking> getHardware() {
    return List.of();
  }
}
