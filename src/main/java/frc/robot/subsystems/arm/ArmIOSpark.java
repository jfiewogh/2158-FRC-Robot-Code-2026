package frc.robot.subsystems.arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class ArmIOSpark implements ArmIO {
    /* Motor */
    private final SparkFlex motor;
    private final RelativeEncoder encoder;
    private SparkFlexConfig config;

    private int canID = 0;
    private int statorCurrentLimit = 0;
    private boolean brakeMode = false;

    public ArmIOSpark() {
        // Initialize motor controller
        motor = new SparkFlex(canID, MotorType.kBrushless);
        config = new SparkFlexConfig();
        config.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

        // Configure encoder
        encoder = motor.getEncoder();
        encoder.setPosition(0);

        // Set current limits
        config.smartCurrentLimit((int) statorCurrentLimit);

        // Save configuration
        motor.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.currentVoltage = motor.getBusVoltage();
        inputs.position = encoder.getPosition();
        inputs.temperature = motor.getMotorTemperature();
        inputs.velocity = encoder.getVelocity();
        inputs.currentAmps = motor.getOutputCurrent();
    }
}
