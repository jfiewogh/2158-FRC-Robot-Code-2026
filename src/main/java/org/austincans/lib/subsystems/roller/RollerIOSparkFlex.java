package org.austincans.lib.subsystems.roller;

import java.util.ArrayList;
import java.util.List;

import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.SparkFlexIO;
import org.austincans.lib.subsystems.selfcheck.HardwareFault;
import org.austincans.lib.subsystems.selfcheck.SelfChecking;
import org.austincans.lib.subsystems.selfcheck.SelfCheckingSparkFlex;

import edu.wpi.first.math.util.Units;

public class RollerIOSparkFlex implements RollerIO {
    public SparkFlexIO motorIO;
    private SelfCheckingSparkFlex rollerMotorCheck;

    public RollerIOSparkFlex(RollerConfig rollerConfig) {
        motorIO = MotorFactory.createSparkFlex(rollerConfig.motorConfig);
        rollerMotorCheck = new SelfCheckingSparkFlex("Roller Motor", motorIO.getMotor());
    }

    @Override
    public void setVelocity(double velocityRPM) {
        motorIO.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM));
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        motorIO.updateInputs(inputs.motorInputs);
    }

    @Override
    public List<SelfChecking> getHardware() {
        return List.of(rollerMotorCheck);
    }

    @Override
    public List<HardwareFault> checkForFaults() {
        List<HardwareFault> faults = new ArrayList<>();

        List<HardwareFault> deployFault = rollerMotorCheck.checkForFaults();
        if (deployFault != null) {
            faults.addAll(deployFault);
        }

        if (faults.isEmpty()) {
            return List.of();
        } else {
            return faults;
        }
    }

    @Override
    public void setVoltage(double voltage) {
        motorIO.setVoltage(voltage);
    }

    

}
