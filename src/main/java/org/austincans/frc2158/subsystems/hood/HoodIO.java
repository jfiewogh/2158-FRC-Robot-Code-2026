package org.austincans.frc2158.subsystems.hood;
import org.austincans.frc2158.math.Angle;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public interface HoodIO {
    @AutoLog
    public class hoodIOInputs {
        public Angle hoodAngleDegrees = Angle.fromDegrees(0);
        public double temperature;
        public double motorPosition;
        public double hoodRadsPerSec;
    }

    public default void updateInputs(hoodIOInputs inputs) {}

    public Angle getAngle();
    
    default void setHoodAngle(Angle setPointDegrees) {}

    default void runHoodMotor(double speed) {}

    default void applyVolts(double volts) {}

    public void setAngleRotation2d(Rotation2d angle);

    public Rotation2d getAngleRotation2d();

    public boolean atAngle(Angle setPoint);

}
