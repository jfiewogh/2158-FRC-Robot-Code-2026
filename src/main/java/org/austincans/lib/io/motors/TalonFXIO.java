package org.austincans.lib.io.motors;

import org.austincans.lib.io.motors.TalonFXConfig.FeedbackConfig;
import org.austincans.lib.io.motors.TalonIO.TalonIOConfigRequests;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXIO extends TalonIO<TalonFXIO.TalonFXIOConfigRequests, TalonFXConfig> {
    private final TalonFX motor;

    private final TalonFXConfiguration vendorConfig = new TalonFXConfiguration();

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> temperature;

    protected TalonFXIO(int deviceId, CANBus canBus) {
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
    public void applyConfigRequests(TalonFXIOConfigRequests configRequests) {
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
            updateMotorOutputConfigs(vendorConfig.MotorOutput, config.motorOutputConfig);

            configRequests.softLimitConfigChanged = false;
            updateSoftLimitSwitchConfigs(vendorConfig.SoftwareLimitSwitch, config.softLimitConfig);

            configRequests.slot0ConfigChanged = false;
            mapSlot0Config(vendorConfig.Slot0, config.slot0Config);

            configRequests.slot1ConfigChanged = false;
            mapSlot1Config(vendorConfig.Slot1, config.slot1Config);

            configRequests.slot2ConfigChanged = false;
            mapSlot2Config(vendorConfig.Slot2, config.slot2Config);

            configRequests.feedbackConfigChanged = false;
            updateFeedbackConfig(vendorConfig.Feedback, config.feedbackConfig);

            configRequests.closedLoopGeneralConfigChanged = false;
            updateClosedLoopGeneralConfig(vendorConfig.ClosedLoopGeneral, config.closedLoopGeneralConfig);

            configRequests.motionMagicConfigChanged = false;
            updateMotionMagicConfig(vendorConfig.MotionMagic, config.motionMagicConfig);

            configRequests.currentLimitsConfigChanged = false;
            updateCurrentLimitsConfig(vendorConfig.CurrentLimits, config.currentLimitsConfig);

            motor.getConfigurator().apply(vendorConfig);
            return;
        }

        /* Individual Configs */
        if (configRequests.motorOutputConfigChanged) {
            configRequests.motorOutputConfigChanged = false;
            updateMotorOutputConfigs(vendorConfig.MotorOutput, config.motorOutputConfig);
            motor.getConfigurator().apply(vendorConfig.MotorOutput);
            return;
        }
        if (configRequests.softLimitConfigChanged) {
            configRequests.softLimitConfigChanged = false;
            updateSoftLimitSwitchConfigs(vendorConfig.SoftwareLimitSwitch, config.softLimitConfig);
            motor.getConfigurator().apply(vendorConfig.SoftwareLimitSwitch);
            return;
        }
        if (configRequests.slot0ConfigChanged) {
            configRequests.slot0ConfigChanged = false;
            mapSlot0Config(vendorConfig.Slot0, config.slot0Config);
            motor.getConfigurator().apply(vendorConfig.Slot0);
            return;
        }
        if (configRequests.slot1ConfigChanged) {
            configRequests.slot1ConfigChanged = false;
            mapSlot1Config(vendorConfig.Slot1, config.slot1Config);
            motor.getConfigurator().apply(vendorConfig.Slot1);
            return;
        }
        if (configRequests.slot2ConfigChanged) {
            configRequests.slot2ConfigChanged = false;
            mapSlot2Config(vendorConfig.Slot2, config.slot2Config);
            motor.getConfigurator().apply(vendorConfig.Slot2);
            return;
        }
        if (configRequests.feedbackConfigChanged) {
            configRequests.feedbackConfigChanged = false;
            updateFeedbackConfig(vendorConfig.Feedback, config.feedbackConfig);
            motor.getConfigurator().apply(vendorConfig.Feedback);
            return;
        }
        if (configRequests.closedLoopGeneralConfigChanged) {
            configRequests.closedLoopGeneralConfigChanged = false;
            updateClosedLoopGeneralConfig(vendorConfig.ClosedLoopGeneral, config.closedLoopGeneralConfig);
            motor.getConfigurator().apply(vendorConfig.ClosedLoopGeneral);
            return;
        }
        if (configRequests.motionMagicConfigChanged) {
            configRequests.motionMagicConfigChanged = false;
            updateMotionMagicConfig(vendorConfig.MotionMagic, config.motionMagicConfig);
            motor.getConfigurator().apply(vendorConfig.MotionMagic);
            return;
        }
        if (configRequests.currentLimitsConfigChanged) {
            configRequests.currentLimitsConfigChanged = false;
            updateCurrentLimitsConfig(vendorConfig.CurrentLimits, config.currentLimitsConfig);
            motor.getConfigurator().apply(vendorConfig.CurrentLimits);
            return;
        }
    }

    /** 
     * See this page for more details on remote sensors
     * https://v6.docs.ctr-electronics.com/en/2025/docs/api-reference/device-specific/talonfx/remote-sensors.html 
     * TODO: Gearbox parameters are temporary. They should become fields of FeedbackConfig.
     */
    private void updateFeedbackConfig(FeedbackConfigs feedback, FeedbackConfig feedbackConfig) {
        switch (feedbackConfig.feedbackSensorSource) {
            case ROTOR -> {
                feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                feedback.SensorToMechanismRatio = feedbackConfig.sensorToMechanismGearbox.getReduction();
            }
            case REMOTE_CANCODER -> {
                feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                feedback.FeedbackRemoteSensorID = feedbackConfig.feedbackSensorSourceId;
                feedback.RotorToSensorRatio = feedbackConfig.rotorToRemoteSensorGearbox.getReduction();
                feedback.SensorToMechanismRatio = feedbackConfig.sensorToMechanismGearbox.getReduction();
            }
            case FUSED_CANCODER -> {
                feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                feedback.FeedbackRemoteSensorID = feedbackConfig.feedbackSensorSourceId;
                feedback.RotorToSensorRatio = feedbackConfig.rotorToRemoteSensorGearbox.getReduction();
                feedback.SensorToMechanismRatio = feedbackConfig.sensorToMechanismGearbox.getReduction();
            }
        }
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

    public static class TalonFXIOConfigRequests extends TalonIOConfigRequests<TalonFXConfig> {
        protected boolean feedbackConfigChanged;

        public void setFeedbackConfig(FeedbackConfig feedbackConfig) {
			configChanged |= feedbackConfigChanged = !config.feedbackConfig.equals(feedbackConfig);
			config.feedbackConfig = feedbackConfig;
		}
    }
}