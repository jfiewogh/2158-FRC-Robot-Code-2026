package org.austincans.lib.io.motors;

import static edu.wpi.first.units.Units.Rotations;

import org.austincans.lib.io.motors.MotorConfig.ControlConfig;
import org.austincans.lib.io.motors.MotorConfig.SoftLimitConfig;
import org.austincans.lib.io.motors.TalonConfig.FeedbackConfig;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public abstract class TalonIO<
	TalonConfigType extends TalonConfig<FeedbackSensorSourceType>, 
	FeedbackSensorSourceType extends TalonConfig.FeedbackSensorSource>
extends BaseMotorIO<
	TalonIO.TalonIOInputs, 
	TalonIO.TalonControlSlot, 
	TalonIO.TalonVelocityControlType, 
	TalonIO.TalonPositionControlType, 
	TalonIO.TalonIOConfigRequests<TalonConfigType, FeedbackSensorSourceType>, 
	TalonConfigType> {

	/* Control Requests */
	protected final VoltageOut voltageOut = new VoltageOut(0);

	private final PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0);
	private final PositionVoltage positionVoltage = new PositionVoltage(0);
	private final PositionTorqueCurrentFOC positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0);
	private final MotionMagicDutyCycle positionMotionMagicDutyCycle = new MotionMagicDutyCycle(0);
	private final MotionMagicVoltage positionMotionMagicVoltage = new MotionMagicVoltage(0);
	private final MotionMagicTorqueCurrentFOC positionMotionMagicTorqueCurrentFOC = new MotionMagicTorqueCurrentFOC(0);
	private final MotionMagicExpoDutyCycle positionMotionMagicExpoDutyCycle = new MotionMagicExpoDutyCycle(0);
	private final MotionMagicExpoVoltage positionMotionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
	private final MotionMagicExpoTorqueCurrentFOC positionMotionMagicExpoTorqueCurrentFOC = new MotionMagicExpoTorqueCurrentFOC(
			0);

	private final VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);
	private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
	private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
	private final MotionMagicVelocityDutyCycle velocityMotionMagicDutyCycle = new MotionMagicVelocityDutyCycle(0);
	private final MotionMagicVelocityVoltage velocityMotionMagicVoltage = new MotionMagicVelocityVoltage(0);
	private final MotionMagicVelocityTorqueCurrentFOC velocityMotionMagicTorqueCurrentFOC = new MotionMagicVelocityTorqueCurrentFOC(
			0);

	public static class TalonIOInputs extends MotorIO.MotorIOInputs {
		private Current supplyCurrent;
		private Current torqueCurrent;

		void setSupplyCurrent(Current supplyCurrent) {
			this.supplyCurrent = supplyCurrent;
		}

		void setTorqueCurrent(Current torqueCurrent) {
			this.torqueCurrent = torqueCurrent;
		}

		public Current getSupplyCurrent() {
			return supplyCurrent;
		}

		public Current getTorqueCurrent() {
			return torqueCurrent;
		}
	}

	@Override
	public void updateInputs(TalonIOInputs inputs) {
		super.updateInputs(inputs);
		inputs.setSupplyCurrent(getSupplyCurrent());
		inputs.setTorqueCurrent(getTorqueCurrent());
	}

	protected abstract Current getSupplyCurrent();

	protected abstract Current getTorqueCurrent();

	protected final ControlRequest getVelocityControlRequest(TalonVelocityControlType velocityControlType,
			AngularVelocity velocity, TalonControlSlot slot, ArbitraryFeedforward arbitraryFeedforward) {
		return switch (velocityControlType) {
			case DUTY_CYCLE -> velocityDutyCycle.withVelocity(velocity).withFeedForward(arbitraryFeedforward.dutyCycle).withSlot(slot.get());
			case VOLTAGE -> velocityVoltage.withVelocity(velocity).withFeedForward(arbitraryFeedforward.volts).withSlot(slot.get());
			case TORQUE_CURRENT_FOC -> velocityTorqueCurrentFOC.withVelocity(velocity).withFeedForward(arbitraryFeedforward.currentAmps).withSlot(slot.get());
			case MOTION_MAGIC_DUTY_CYCLE -> velocityMotionMagicDutyCycle
				.withVelocity(velocity)
				.withFeedForward(arbitraryFeedforward.dutyCycle)
				.withSlot(slot.get());
			case MOTION_MAGIC_VOLTAGE -> velocityMotionMagicVoltage
				.withVelocity(velocity)
				.withFeedForward(arbitraryFeedforward.volts)
				.withSlot(slot.get());
			case MOTION_MAGIC_TORQUE_CURRENT_FOC -> velocityMotionMagicTorqueCurrentFOC
				.withVelocity(velocity)
				.withFeedForward(arbitraryFeedforward.currentAmps)
				.withSlot(slot.get());
		};
	}

	protected final ControlRequest getPositionControlRequest(TalonPositionControlType positionControlType, Angle position,
			TalonControlSlot slot, AngularVelocity velocity, ArbitraryFeedforward arbitraryFeedforward) {
		return switch (positionControlType) {
			case DUTY_CYCLE -> positionDutyCycle
				.withPosition(position)
				.withVelocity(velocity)
				.withFeedForward(arbitraryFeedforward.dutyCycle)
				.withSlot(slot.get());
			case VOLTAGE -> positionVoltage
				.withPosition(position)
				.withVelocity(velocity)
				.withFeedForward(arbitraryFeedforward.volts)
				.withSlot(slot.get());
			case TORQUE_CURRENT_FOC -> positionTorqueCurrentFOC
				.withPosition(position)
				.withVelocity(velocity)
				.withFeedForward(arbitraryFeedforward.currentAmps)
				.withSlot(slot.get());
			case MOTION_MAGIC_DUTY_CYCLE -> positionMotionMagicDutyCycle
				.withPosition(position)
				.withFeedForward(arbitraryFeedforward.dutyCycle)
				.withSlot(slot.get());
			case MOTION_MAGIC_VOLTAGE -> positionMotionMagicVoltage
				.withPosition(position)
				.withFeedForward(arbitraryFeedforward.volts)
				.withSlot(slot.get());
			case MOTION_MAGIC_TORQUE_CURRENT_FOC -> positionMotionMagicTorqueCurrentFOC
				.withPosition(position)
				.withFeedForward(arbitraryFeedforward.currentAmps)
				.withSlot(slot.get());
			case MOTION_MAGIC_EXPO_DUTY_CYCLE -> positionMotionMagicExpoDutyCycle
				.withPosition(position)
				.withFeedForward(arbitraryFeedforward.dutyCycle)
				.withSlot(slot.get());
			case MOTION_MAGIC_EXPO_VOLTAGE -> positionMotionMagicExpoVoltage
				.withPosition(position)
				.withFeedForward(arbitraryFeedforward.volts)
				.withSlot(slot.get());
			case MOTION_MAGIC_EXPO_TORQUE_CURRENT_FOC -> positionMotionMagicExpoTorqueCurrentFOC
				.withPosition(position)
				.withFeedForward(arbitraryFeedforward.currentAmps)
				.withSlot(slot.get());
		};
	}

	protected static int getTrueCount(final boolean... array) {
		int trueCount = 0;
		for (boolean bool : array) {
			if (bool)
				trueCount++;
		}
		return trueCount;
	}



	/*===================== Configs =====================*/

	protected void updateMotorOutputConfigs(MotorOutputConfigs motorOutput, boolean inverted, NeutralMode neutralMode) {
        motorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorOutput.NeutralMode = neutralMode == NeutralMode.BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    }

    protected void updateSoftLimitSwitchConfigs(SoftwareLimitSwitchConfigs softLimit, SoftLimitConfig softLimitConfig) {
        softLimit.ForwardSoftLimitEnable = softLimitConfig.forwardSoftLimitEnabled;
        softLimit.ForwardSoftLimitThreshold = softLimitConfig.forwardSoftLimit.in(Rotations);
        softLimit.ReverseSoftLimitEnable = softLimitConfig.reverseSoftLimitEnabled;
        softLimit.ReverseSoftLimitThreshold = softLimitConfig.reverseSoftLimit.in(Rotations);
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



	/*===================== Classes =====================*/

	public static class TalonIOConfigRequests<TalonConfigType extends TalonConfig<FeedbackSensorSourceType>, FeedbackSensorSourceType extends TalonConfig.FeedbackSensorSource>
			extends MotorIO.MotorIOConfigRequests<TalonConfigType> {

		protected boolean softLimitConfigChanged;
		protected boolean motorOutputConfigChanged;
		protected boolean slot0ConfigChanged;
		protected boolean slot1ConfigChanged;
		protected boolean slot2ConfigChanged;
		protected boolean feedbackConfigChanged;
		protected boolean closedLoopGeneralConfigChanged;
		protected boolean motionMagicConfigChanged;
		protected boolean currentLimitsConfigChanged;

		public void setSoftLimitConfig(SoftLimitConfig softLimitConfig) {
			configChanged |= softLimitConfigChanged = !config.mechanismSoftLimitConfig.equals(softLimitConfig);
			config.mechanismSoftLimitConfig = softLimitConfig;
		}

		/* TODO: Merge setNeutralMode and setInverted into setMotorOutputConfig */
		public void setNeutralMode(NeutralMode neutralMode) {
			configChanged |= motorOutputConfigChanged |= config.neutralMode != neutralMode;
			config.neutralMode = neutralMode;
		}
		public void setInverted(boolean inverted) {
			configChanged |= motorOutputConfigChanged |= config.inverted != inverted;
			config.inverted = inverted;
		}

		public void setSlot0Config(ControlConfig slotConfig) {
			configChanged |= slot0ConfigChanged = !config.slot0Config.equals(slotConfig);
			config.slot0Config = slotConfig;
		}

		public void setSlot1Config(ControlConfig slotConfig) {
			configChanged |= slot1ConfigChanged = !config.slot1Config.equals(slotConfig);
			config.slot1Config = slotConfig;
		}

		public void setSlot2Config(ControlConfig slotConfig) {
			configChanged |= slot2ConfigChanged = !config.slot2Config.equals(slotConfig);
			config.slot2Config = slotConfig;
		}

		public void setFeedbackConfig(FeedbackConfig<FeedbackSensorSourceType> feedbackConfig) {
			configChanged |= feedbackConfigChanged = config.feedbackConfig.equals(feedbackConfig);
			config.feedbackConfig = feedbackConfig;
		}

		public void setPositionContinuousWrap(boolean continuousWrap) {
			configChanged |= closedLoopGeneralConfigChanged |= config.positionContinuousWrap != continuousWrap;
			config.positionContinuousWrap = continuousWrap;
		}

		/* TODO: Merge into setMotionMagicConfig */
		public void setMotionMagicCruiseVelocity(AngularVelocity velocity) {
			configChanged |= motionMagicConfigChanged |= !config.motionMagicMechanismCruiseVelocity.isEquivalent(velocity);
			config.motionMagicMechanismCruiseVelocity = velocity;
		}
		public void setMotionMagicAcceleration(AngularAcceleration acceleration) {
			configChanged |= motionMagicConfigChanged |= !config.motionMagicMechanismAcceleration.isEquivalent(acceleration);
			config.motionMagicMechanismAcceleration = acceleration;
		}
		public void setMotionMagicJerk(double jerkRotationsPerSecCubed) {
			configChanged |= motionMagicConfigChanged |= config.motionMagicMechanismJerkRotationsPerSecCubed != jerkRotationsPerSecCubed;
			config.motionMagicMechanismJerkRotationsPerSecCubed = jerkRotationsPerSecCubed;
		}

		/* TODO: Merge into setCurrentLimitsConfig */
		public void setStatorCurrentLimit(boolean enabled, Current current) {
			configChanged |= currentLimitsConfigChanged |= (config.currentLimitsConfig.statorCurrentLimitEnabled != enabled
					|| !config.currentLimitsConfig.statorCurrentLimit.isEquivalent(current));
			config.currentLimitsConfig.statorCurrentLimitEnabled = enabled;
			config.currentLimitsConfig.statorCurrentLimit = current;
		}
		public void setSupplyCurrentLimit(boolean enabled, Current current) {
			configChanged |= currentLimitsConfigChanged |= (config.currentLimitsConfig.supplyCurrentLimitEnabled != enabled
					|| !config.currentLimitsConfig.supplyCurrentLimit.isEquivalent(current));
			config.currentLimitsConfig.supplyCurrentLimitEnabled = enabled;
			config.currentLimitsConfig.supplyCurrentLimit = current;
		}
	}

	public static enum TalonVelocityControlType implements MotorIO.VelocityControlType {
		DUTY_CYCLE,
		VOLTAGE,
		TORQUE_CURRENT_FOC,
		MOTION_MAGIC_DUTY_CYCLE,
		MOTION_MAGIC_VOLTAGE,
		MOTION_MAGIC_TORQUE_CURRENT_FOC,
	}

	public static enum TalonPositionControlType implements MotorIO.PositionControlType {
		DUTY_CYCLE,
		VOLTAGE,
		TORQUE_CURRENT_FOC,
		MOTION_MAGIC_DUTY_CYCLE,
		MOTION_MAGIC_VOLTAGE,
		MOTION_MAGIC_TORQUE_CURRENT_FOC,
		MOTION_MAGIC_EXPO_DUTY_CYCLE,
		MOTION_MAGIC_EXPO_VOLTAGE,
		MOTION_MAGIC_EXPO_TORQUE_CURRENT_FOC;
	}

	public static enum TalonControlSlot implements MotorIO.ControlSlot {
		SLOT_0,
		SLOT_1,
		SLOT_2;

		public int get() {
			return this.ordinal();
		}
	}
}