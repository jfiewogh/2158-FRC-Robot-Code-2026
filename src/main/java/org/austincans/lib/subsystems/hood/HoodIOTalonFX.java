package org.austincans.lib.subsystems.hood;

import java.util.ArrayList;
import java.util.List;

import org.austincans.lib.math.Angle;
import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.MotorIOTalonFX;
import org.austincans.lib.subsystems.selfcheck.HardwareFault;
import org.austincans.lib.subsystems.selfcheck.SelfCheckingTalonFXPro;
import org.austincans.lib.subsystems.selfcheck.SelfCheckingTalonFXS;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class HoodIOTalonFX implements HoodIO {
    private MotorIOTalonFX motorIO;
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private SelfCheckingTalonFXPro hoodMotorCheck;

    public HoodIOTalonFX(HoodConfig hoodConfig){
        motorIO = MotorFactory.createTalonFX(hoodConfig.deviceID(), hoodConfig.motorCanbus(), config, hoodConfig.pidffConfig(), hoodConfig.softLimitConfig(), hoodConfig.nuetralMode(), hoodConfig.isInverted());
        hoodMotorCheck = new SelfCheckingTalonFXPro("Hood Motor", motorIO.getMotor());
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodAngleDeg = Units.radiansToDegrees(motorIO.getPositionRad());
        inputs.motorPositionRad = motorIO.getPositionRad();
        inputs.temperatureCelsius = motorIO.getMotor().getDeviceTemp().getValueAsDouble();
        inputs.velocityRadPerSec = motorIO.getVelocityRadPerSec();
    }

    @Override
    public void setAngle(Angle angle) {
        motorIO.setPosition(angle.toRadians());
    }

    @Override
    public void setAngleRotation2d(Rotation2d angle) {
        motorIO.setPosition(angle.getRadians());
    }

    @Override
    public Angle getAngle() {
        return Angle.fromRadians(motorIO.getPositionRad());
    }

    @Override
    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromRotations(motorIO.getPositionRad());
    }

    @Override
    public boolean atAngle(Angle setpoint) {
        return getAngle() == setpoint;
    }

    @Override
    public void setVoltage(double voltage) {
        motorIO.setVoltage(voltage);
    }

    @Override
    public List<HardwareFault> checkForFaults() {
        List<HardwareFault> faults = new ArrayList<>();

        List<HardwareFault> deployFault = hoodMotorCheck.checkForFaults();
        if (deployFault != null) {
            faults.addAll(deployFault);
        }

        if (faults.isEmpty()) {
            return null;
        } else {
            return faults;
        }
    }

}
