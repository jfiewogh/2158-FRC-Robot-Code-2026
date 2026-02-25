package org.austincans.frc2158.subsystems.turret;

import java.util.ArrayList;
import java.util.List;
import org.austincans.frc2158.subsystems.selfcheck.HardwareFault;
import org.austincans.frc2158.subsystems.selfcheck.SelfChecking;
import org.austincans.frc2158.subsystems.selfcheck.SelfCheckingSparkFlex;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class TurretIOSparkFlex extends TurretIOReal {
    private final SparkFlex motor;
    private final SparkClosedLoopController controller;
    private final SelfCheckingSparkFlex selfCheckingMotor;

    public TurretIOSparkFlex() {
        super();
        motor = new SparkFlex(TurretConstants.turretMotorId,
                TurretConstants.isBrushlessMotor ? MotorType.kBrushless : MotorType.kBrushed);
        selfCheckingMotor = new SelfCheckingSparkFlex("Turret", motor);

        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig.closedLoop.pid(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
        motorConfig.closedLoop.feedForward.kS(TurretConstants.kS);
        motorConfig.closedLoop.feedForward.kV(TurretConstants.kV);
        motorConfig.closedLoop.feedForward.kA(TurretConstants.kA);
        motorConfig.softLimit.forwardSoftLimitEnabled(TurretConstants.forwardSoftLimitEnabled);
        motorConfig.softLimit.forwardSoftLimit(TurretConstants.forwardSoftLimit);
        motorConfig.softLimit.reverseSoftLimitEnabled(TurretConstants.reverseSoftLimitEnabled);
        motorConfig.softLimit.reverseSoftLimit(TurretConstants.reverseSoftLimit);
        motorConfig.smartCurrentLimit(TurretConstants.smartCurrentLimit);
        motorConfig.idleMode(TurretConstants.isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);   

        controller = motor.getClosedLoopController();
    }

    @Override
    public void setVoltage(double voltage) {
        controller.setSetpoint(voltage, ControlType.kVoltage);
    }

    @Override
    public void setDesiredAngle(Rotation2d angle) {
        controller.setSetpoint(angle.getRotations(), ControlType.kPosition);
    }

    @Override
    public void setVelocity(double radiansPerSec) {
        controller.setSetpoint(Units.radiansToRotations(radiansPerSec), ControlType.kVelocity);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        super.updateInputs(inputs);
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.voltage = motor.getBusVoltage();
        inputs.temperatureCelsius = motor.getMotorTemperature();
    }

    @Override
    public List<HardwareFault> checkForFaults() {
        List<HardwareFault> faults = new ArrayList<>();
        List<HardwareFault> deployFaults = selfCheckingMotor.checkForFaults();
        if (deployFaults != null) {
            faults.addAll(deployFaults);
        }
        return faults.isEmpty() ? null : faults;
    }

    @Override
    public List<SelfChecking> getHardware() {
        return List.of(selfCheckingMotor);
    }
}
