package org.austincans.frc2026.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;

public class HoodUtil {
    public static Rotation2d pwmToAngle(int pulseWidth) {
        return Rotation2d.fromDegrees(HoodConstants.pwmToAngle.get((double) pulseWidth));
    }

    public static int angleToPWM(Rotation2d angle) {
        return HoodConstants.angleToPWM.get(angle.getDegrees()).intValue();
    }
}
