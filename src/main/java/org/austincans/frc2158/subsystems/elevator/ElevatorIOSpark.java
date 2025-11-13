package org.austincans.frc2158.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ElevatorIOSpark implements ElevatorIO {
    private int canID = 0;
    private final double maxVelocity = 1; // meters per second
    private final double maxAcceleration = 1; // meters per second squared
    private final double drumRadius = 0.0254; // meters
    private final boolean brakeMode = true;

    private final SparkFlex motor;
    private final RelativeEncoder encoder;
    private SparkFlexConfig config;

    private TrapezoidProfile.Constraints constraints;


    @SuppressWarnings("unused")
    private final boolean enableStatorLimit = true;
  
    private final double statorCurrentLimit = 40;
  
    @SuppressWarnings("unused")
    private final boolean enableSupplyLimit = false;
  
    @SuppressWarnings("unused")
    private final double supplyCurrentLimit = 40;

    public ElevatorIOSpark(){
        motor = new SparkFlex(canID, MotorType.kBrushless);
        config = new SparkFlexConfig();
        config.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

        // Configure encoder
        encoder = motor.getEncoder();
        encoder.setPosition(0);

        // Set current limits
        config.smartCurrentLimit((int) statorCurrentLimit);

        // Save configuration
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize ProfiledPIDController
        // Convert from meters to rotations for constraints
        double maxVelocityRotations = maxVelocity / (2.0 * Math.PI * drumRadius);
        double maxAccelerationRotations = maxAcceleration / (2.0 * Math.PI * drumRadius);

        constraints = new TrapezoidProfile.Constraints(maxVelocityRotations, maxAccelerationRotations);
    }
}
