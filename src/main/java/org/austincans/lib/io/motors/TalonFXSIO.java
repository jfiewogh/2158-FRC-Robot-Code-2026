package org.austincans.lib.io.motors;

import org.austincans.lib.io.motors.TalonFXSConfig.ExternalFeedbackConfig;
import org.austincans.lib.util.math.physical.Gearbox;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXSIO extends TalonIO<TalonFXSIO.TalonFXSIOConfigRequests, TalonFXSConfig> {
    private final TalonFXS motor;

    private final TalonFXSConfiguration vendorConfig = new TalonFXSConfiguration();

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> temperature;

    TalonFXSIO(int deviceId, CANBus canBus, TalonFXConfig config) {
        motor = new TalonFXS(deviceId, canBus);

        position = motor.getPosition();
        velocity = motor.getVelocity();
        voltage = motor.getMotorVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        statorCurrent = motor.getStatorCurrent();
        torqueCurrent = motor.getTorqueCurrent();
        temperature = motor.getDeviceTemp();
    }

    @Override
    public void applyConfigRequests(TalonFXSIOConfigRequests configRequests) {
        if (!configRequests.configChanged) {
            return;
        }
        configRequests.configChanged = false;

        TalonFXSConfig config = configRequests.config;

        int changeCount = getTrueCount(
            configRequests.motorOutputConfigChanged, 
            configRequests.softLimitConfigChanged,
            configRequests.slot0ConfigChanged,
            configRequests.slot1ConfigChanged,
            configRequests.slot2ConfigChanged,
            configRequests.externalFeedbackConfigChanged,
            configRequests.closedLoopGeneralConfigChanged,
            configRequests.motionMagicConfigChanged,
            configRequests.currentLimitsConfigChanged
        );

        /* TODO: There are definitely some commonalities between TalonFXIO and TalonFXSIO. Separate common parts into shared methods */

        /* Multiple Configs */
        if (changeCount > 1) {
            configRequests.motorOutputConfigChanged = false;
            updateMotorOutputConfigs(vendorConfig.MotorOutput, config.inverted, config.neutralMode);

            configRequests.softLimitConfigChanged = false;
            updateSoftLimitSwitchConfigs(vendorConfig.SoftwareLimitSwitch, config.mechanismSoftLimitConfig);

            configRequests.slot0ConfigChanged = false;
            mapSlot0Config(vendorConfig.Slot0, config.slot0Config);

            configRequests.slot1ConfigChanged = false;
            mapSlot1Config(vendorConfig.Slot1, config.slot1Config);

            configRequests.slot2ConfigChanged = false;
            mapSlot2Config(vendorConfig.Slot2, config.slot2Config);

            configRequests.externalFeedbackConfigChanged = false;
            updateExternalFeedbackConfig(vendorConfig.ExternalFeedback, config.externalFeedbackConfig, config.sensorToMechanismGearbox, config.rotorToRemoteSensor);

            configRequests.closedLoopGeneralConfigChanged = false;
            updateClosedLoopGeneralConfig(vendorConfig.ClosedLoopGeneral, config.positionContinuousWrap);

            configRequests.motionMagicConfigChanged = false;
            updateMotionMagicConfig(vendorConfig.MotionMagic, config.motionMagicMechanismCruiseVelocity, config.motionMagicMechanismAcceleration, config.motionMagicMechanismJerkRotationsPerSecCubed);

            configRequests.currentLimitsConfigChanged = false;
            updateCurrentLimitsConfig(vendorConfig.CurrentLimits, config.currentLimitsConfig);

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
        if (configRequests.externalFeedbackConfigChanged) {
            configRequests.externalFeedbackConfigChanged = false;
            updateExternalFeedbackConfig(vendorConfig.ExternalFeedback, config.externalFeedbackConfig, config.sensorToMechanismGearbox, config.rotorToRemoteSensor);
            motor.getConfigurator().apply(vendorConfig.ExternalFeedback);
            return;
        }
        if (configRequests.closedLoopGeneralConfigChanged) {
            configRequests.closedLoopGeneralConfigChanged = false;
            updateClosedLoopGeneralConfig(vendorConfig.ClosedLoopGeneral, config.positionContinuousWrap);
            motor.getConfigurator().apply(vendorConfig.ClosedLoopGeneral);
            return;
        }
        if (configRequests.motionMagicConfigChanged) {
            configRequests.motionMagicConfigChanged = false;
            updateMotionMagicConfig(vendorConfig.MotionMagic, config.motionMagicMechanismCruiseVelocity, config.motionMagicMechanismAcceleration, config.motionMagicMechanismJerkRotationsPerSecCubed);
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

    protected void updateExternalFeedbackConfig(ExternalFeedbackConfigs externalFeedback, ExternalFeedbackConfig externalFeedbackConfig, Gearbox sensorToMechanismGearbox, Gearbox rotorToRemoteSensor) {
        switch (externalFeedbackConfig.externalFeedbackSensorSource) {
            case COMMUTATION -> {
                // TODO: Check AdvantageKit's swerve template with TalonFXS to see how they handled Commutation
                externalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Commutation;
                externalFeedback.SensorToMechanismRatio = sensorToMechanismGearbox.getReduction();
            }
            case REMOTE_CANCODER -> {
                externalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
                externalFeedback.FeedbackRemoteSensorID = externalFeedbackConfig.externalFeedbackSensorSourceId;
                externalFeedback.RotorToSensorRatio = rotorToRemoteSensor.getReduction();
                externalFeedback.SensorToMechanismRatio = sensorToMechanismGearbox.getReduction();
            }
            case FUSED_CANCODER -> {
                externalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.FusedCANcoder;
                externalFeedback.FeedbackRemoteSensorID = externalFeedbackConfig.externalFeedbackSensorSourceId;
                externalFeedback.RotorToSensorRatio = rotorToRemoteSensor.getReduction();
                externalFeedback.SensorToMechanismRatio = sensorToMechanismGearbox.getReduction();
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

    public static class TalonFXSIOConfigRequests extends TalonIO.TalonIOConfigRequests<TalonFXSConfig> {
        protected boolean externalFeedbackConfigChanged;

        public void setExternalFeedbackConfig(ExternalFeedbackConfig externalFeedbackConfig) {
			configChanged |= externalFeedbackConfigChanged = config.externalFeedbackConfig.equals(externalFeedbackConfig);
			config.externalFeedbackConfig = externalFeedbackConfig;
		}
    }
}