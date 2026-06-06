package org.austincans.lib.io.motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import org.austincans.lib.io.motors.TalonFXConfig.TalonFXFeedbackSensorSource;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXIO extends TalonIO<TalonFXConfig, TalonFXFeedbackSensorSource> {
    private final TalonFX motor;

    private final TalonFXConfiguration vendorConfig = new TalonFXConfiguration();

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> temperature;

    TalonFXIO(int deviceId, CANBus canBus, TalonFXConfig config) {
        motor = new TalonFX(deviceId, canBus);

        position = motor.getPosition();
        velocity = motor.getVelocity();
        voltage = motor.getMotorVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        statorCurrent = motor.getStatorCurrent();
        torqueCurrent = motor.getTorqueCurrent();
        temperature = motor.getDeviceTemp();
    }

    @Override
    public void applyConfigRequests(TalonIOConfigRequests<TalonFXConfig, TalonFXFeedbackSensorSource> configRequests) {
        if (!configRequests.configChanged) {
            return;
        }
        configRequests.configChanged = false;

        TalonFXConfig config = configRequests.config;

        int changeCount = getTrueCount(
            configRequests.motorOutputConfigChanged, 
            configRequests.softLimitConfigChanged,
            configRequests.slot0ConfigChanged,
            configRequests.slot1ConfigChanged,
            configRequests.slot2ConfigChanged,
            configRequests.feedbackConfigChanged,
            configRequests.closedLoopGeneralConfigChanged,
            configRequests.motionMagicConfigChanged,
            configRequests.currentLimitsConfigChanged
        );

        /* Multiple Configs */
        if (changeCount > 1) {
            configRequests.motorOutputConfigChanged = false;
            updateMotorOutputConfigs(vendorConfig.MotorOutput, config.inverted, config.neutralMode);

            configRequests.softLimitConfigChanged = false;
            updateSoftLimitSwitchConfigs(vendorConfig.SoftwareLimitSwitch, config.mechanismSoftLimitConfig);

            configRequests.slot0ConfigChanged = false;
            updateSlot0Configs(config);

            configRequests.slot1ConfigChanged = false;
            updateSlot1Configs(config);

            configRequests.slot2ConfigChanged = false;
            updateSlot2Configs(config);

            configRequests.feedbackConfigChanged = false;
            updateFeedbackConfig(config);

            configRequests.closedLoopGeneralConfigChanged = false;
            updateClosedLoopGeneralConfig(config);

            configRequests.motionMagicConfigChanged = false;
            updateMotionMagicConfig(config);

            configRequests.currentLimitsConfigChanged = false;
            updateCurrentLimitsConfig(config);

            motor.getConfigurator().apply(vendorConfig);
            return;
        }

        /* Individual Configs */
        if (configRequests.motorOutputConfigChanged) {
            configRequests.motorOutputConfigChanged = false;
            updateMotorOutputConfigs(vendorConfig.MotorOutput, config.inverted, config.neutralMode);
            motor.getConfigurator().apply(vendorConfig.MotorOutput);
            return;
        }
        if (configRequests.softLimitConfigChanged) {
            configRequests.softLimitConfigChanged = false;
            updateSoftLimitSwitchConfigs(vendorConfig.SoftwareLimitSwitch, config.mechanismSoftLimitConfig);
            motor.getConfigurator().apply(vendorConfig.SoftwareLimitSwitch);
            return;
        }
        if (configRequests.slot0ConfigChanged) {
            configRequests.slot0ConfigChanged = false;
            updateSlot0Configs(config);
            motor.getConfigurator().apply(vendorConfig.Slot0);
            return;
        }
        if (configRequests.slot1ConfigChanged) {
            configRequests.slot1ConfigChanged = false;
            updateSlot1Configs(config);
            motor.getConfigurator().apply(vendorConfig.Slot1);
            return;
        }
        if (configRequests.slot2ConfigChanged) {
            configRequests.slot2ConfigChanged = false;
            updateSlot2Configs(config);
            motor.getConfigurator().apply(vendorConfig.Slot2);
            return;
        }
        if (configRequests.feedbackConfigChanged) {
            configRequests.feedbackConfigChanged = false;
            updateFeedbackConfig(config);
            motor.getConfigurator().apply(vendorConfig.Feedback);
            return;
        }
        if (configRequests.closedLoopGeneralConfigChanged) {
            configRequests.closedLoopGeneralConfigChanged = false;
            updateClosedLoopGeneralConfig(config);
            motor.getConfigurator().apply(vendorConfig.ClosedLoopGeneral);
            return;
        }
        if (configRequests.motionMagicConfigChanged) {
            configRequests.motionMagicConfigChanged = false;
            updateMotionMagicConfig(config);
            motor.getConfigurator().apply(vendorConfig.MotionMagic);
            return;
        }
        if (configRequests.currentLimitsConfigChanged) {
            configRequests.currentLimitsConfigChanged = false;
            updateCurrentLimitsConfig(config);
            motor.getConfigurator().apply(vendorConfig.CurrentLimits);
            return;
        }
    }

    private void updateSlot0Configs(TalonFXConfig config) {
        vendorConfig.Slot0.kP = config.slot0Config.pidConfig.kP;
        vendorConfig.Slot0.kI = config.slot0Config.pidConfig.kI;
        vendorConfig.Slot0.kD = config.slot0Config.pidConfig.kD;
        vendorConfig.Slot0.kS = config.slot0Config.ffConfig.kS;
        vendorConfig.Slot0.kG = config.slot0Config.ffConfig.kG;
        vendorConfig.Slot0.kV = config.slot0Config.ffConfig.kV;
        vendorConfig.Slot0.kA = config.slot0Config.ffConfig.kA;
    }
    private void updateSlot1Configs(TalonFXConfig config) {
        vendorConfig.Slot1.kP = config.slot1Config.pidConfig.kP;
        vendorConfig.Slot1.kI = config.slot1Config.pidConfig.kI;
        vendorConfig.Slot1.kD = config.slot1Config.pidConfig.kD;
        vendorConfig.Slot1.kS = config.slot1Config.ffConfig.kS;
        vendorConfig.Slot1.kG = config.slot1Config.ffConfig.kG;
        vendorConfig.Slot1.kV = config.slot1Config.ffConfig.kV;
        vendorConfig.Slot1.kA = config.slot1Config.ffConfig.kA;
    }
    private void updateSlot2Configs(TalonFXConfig config) {
        vendorConfig.Slot2.kP = config.slot2Config.pidConfig.kP;
        vendorConfig.Slot2.kI = config.slot2Config.pidConfig.kI;
        vendorConfig.Slot2.kD = config.slot2Config.pidConfig.kD;
        vendorConfig.Slot2.kS = config.slot2Config.ffConfig.kS;
        vendorConfig.Slot2.kG = config.slot2Config.ffConfig.kG;
        vendorConfig.Slot2.kV = config.slot2Config.ffConfig.kV;
        vendorConfig.Slot2.kA = config.slot2Config.ffConfig.kA;
    }

    /** 
     * See this page for more details on remote sensors
     * https://v6.docs.ctr-electronics.com/en/2025/docs/api-reference/device-specific/talonfx/remote-sensors.html 
     */
    private void updateFeedbackConfig(TalonFXConfig config) {
        switch (config.feedbackConfig.feedbackSensorSource) {
            case ROTOR -> {
                vendorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                vendorConfig.Feedback.SensorToMechanismRatio = config.sensorToMechanismGearbox.getReduction();
            }
            case REMOTE_CANCODER -> {
                vendorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                vendorConfig.Feedback.FeedbackRemoteSensorID = config.feedbackConfig.feedbackSensorSourceId;
                vendorConfig.Feedback.RotorToSensorRatio = config.rotorToRemoteSensor.getReduction();
                vendorConfig.Feedback.SensorToMechanismRatio = config.sensorToMechanismGearbox.getReduction();
            }
            case FUSED_CANCODER -> {
                vendorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                vendorConfig.Feedback.FeedbackRemoteSensorID = config.feedbackConfig.feedbackSensorSourceId;
                vendorConfig.Feedback.RotorToSensorRatio = config.rotorToRemoteSensor.getReduction();
                vendorConfig.Feedback.SensorToMechanismRatio = config.sensorToMechanismGearbox.getReduction();
            }
        }
    }

    private void updateClosedLoopGeneralConfig(TalonFXConfig config) {
        vendorConfig.ClosedLoopGeneral.ContinuousWrap = config.positionContinuousWrap;
    }
    private void updateMotionMagicConfig(TalonFXConfig config) {
        vendorConfig.MotionMagic.MotionMagicCruiseVelocity = config.motionMagicMechanismCruiseVelocity.in(RotationsPerSecond);
        vendorConfig.MotionMagic.MotionMagicAcceleration = config.motionMagicMechanismAcceleration.in(RotationsPerSecondPerSecond);
        vendorConfig.MotionMagic.MotionMagicJerk = config.motionMagicMechanismJerkRotationsPerSecCubed;
    }
    private void updateCurrentLimitsConfig(TalonFXConfig config) {
        vendorConfig.CurrentLimits.StatorCurrentLimitEnable = config.currentLimitsConfig.statorCurrentLimitEnabled;
        vendorConfig.CurrentLimits.StatorCurrentLimit = config.currentLimitsConfig.statorCurrentLimit.in(Amps);
        vendorConfig.CurrentLimits.SupplyCurrentLimitEnable = config.currentLimitsConfig.supplyCurrentLimitEnabled;
        vendorConfig.CurrentLimits.SupplyCurrentLimit = config.currentLimitsConfig.supplyCurrentLimit.in(Amps);
    }


    @Override
    protected void setVoltage(Voltage voltage) {
        motor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    protected void setPosition(Angle position, TalonPositionControlType positionControlType, TalonControlSlot slot, AngularVelocity velocity, ArbitraryFeedforward arbitraryFeedforward) {
        motor.setControl(getPositionControlRequest(positionControlType, position, slot, velocity, arbitraryFeedforward));
    }

    @Override
    protected void setVelocity(AngularVelocity velocity, TalonVelocityControlType velocityControlType, TalonControlSlot slot, ArbitraryFeedforward arbitraryFeedforward) {
        motor.setControl(getVelocityControlRequest(velocityControlType, velocity, slot, arbitraryFeedforward)); 
    }

    @Override
    protected Angle getPosition() {
        return position.getValue();
    }

    @Override
    protected AngularVelocity getVelocity() {
        return velocity.getValue();
    }

    @Override
    protected Voltage getVoltage() {
        return voltage.getValue();
    }

    @Override
    protected Current getStatorCurrent() {
        return statorCurrent.getValue();
    }

    @Override
    protected Current getSupplyCurrent() {
        return supplyCurrent.getValue();
    }

    @Override
    protected Current getTorqueCurrent() {
        return torqueCurrent.getValue();
    }

    @Override
    protected Temperature getTemperature() {
        return temperature.getValue();
    }

    @Override
    public void setEncoderPosition(Angle position) {
        motor.setPosition(position);
    }
}