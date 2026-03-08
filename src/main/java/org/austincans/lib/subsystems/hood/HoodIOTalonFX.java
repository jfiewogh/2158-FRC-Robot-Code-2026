package org.austincans.lib.subsystems.hood;

import java.util.ArrayList;
import java.util.List;

import org.austincans.lib.math.Angle;
import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.MotorIOTalonFX;
import org.austincans.lib.subsystems.selfcheck.HardwareFault;
import org.austincans.lib.subsystems.selfcheck.SelfCheckingTalonFXPro;
import edu.wpi.first.math.geometry.Rotation2d;

public class HoodIOTalonFX implements HoodIO {
    private MotorIOTalonFX motorIO;
    private SelfCheckingTalonFXPro hoodMotorCheck;

    public HoodIOTalonFX(HoodConfig hoodConfig){
        motorIO = MotorFactory.createTalonFX(hoodConfig.motorConfig());
        hoodMotorCheck = new SelfCheckingTalonFXPro("Hood Motor", motorIO.getMotor());
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        motorIO.updateInputs(inputs.motorInputs);
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
        return Angle.fromRotations(motorIO.getMotor().getPosition().getValueAsDouble());
    }

    @Override
    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromRotations(motorIO.getMotor().getPosition().getValueAsDouble());
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
