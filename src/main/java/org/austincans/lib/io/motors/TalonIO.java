package org.austincans.lib.io.motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import org.austincans.lib.io.motors.MotorConfigs.ControlConfig;
import org.austincans.lib.io.motors.MotorConfigs.SoftLimitConfig;
import org.austincans.lib.io.motors.TalonConfig.ClosedLoopGeneralConfig;
import org.austincans.lib.io.motors.TalonConfig.CurrentLimitsConfig;
import org.austincans.lib.io.motors.TalonConfig.MotionMagicConfig;
import org.austincans.lib.io.motors.TalonConfig.MotorOutputConfig;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public abstract class TalonIO<
	TalonConfigRequestsType extends TalonIO.TalonIOConfigRequests<TalonConfigType>,
	TalonConfigType extends TalonConfig>
extends BaseMotorIO<
	TalonIO.TalonIOInputs, 
	TalonIO.TalonControlSlot, 
	TalonIO.TalonVelocityControlType, 
	TalonIO.TalonPositionControlType, 
	TalonConfigRequestsType, 
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
		protected Current supplyCurrent;
		protected Current torqueCurrent;

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
		inputs.supplyCurrent = getSupplyCurrent();
		inputs.torqueCurrent = getTorqueCurrent();
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

	protected void updateMotorOutputConfigs(MotorOutputConfigs motorOutput, MotorOutputConfig motorOutputConfig) {
        motorOutput.Inverted = motorOutputConfig.inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorOutput.NeutralMode = motorOutputConfig.neutralMode == NeutralMode.BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    }
    protected void updateSoftLimitSwitchConfigs(SoftwareLimitSwitchConfigs softLimit, SoftLimitConfig softLimitConfig) {
        softLimit.ForwardSoftLimitEnable = softLimitConfig.forwardSoftLimitEnabled;
        softLimit.ForwardSoftLimitThreshold = softLimitConfig.forwardSoftLimit.in(Rotations);
        softLimit.ReverseSoftLimitEnable = softLimitConfig.reverseSoftLimitEnabled;
        softLimit.ReverseSoftLimitThreshold = softLimitConfig.reverseSoftLimit.in(Rotations);
	}
    protected void mapSlot0Config(Slot0Configs slot0, ControlConfig slot0Config) {
        slot0.kP = slot0Config.pidConfig.kP;
        slot0.kI = slot0Config.pidConfig.kI;
        slot0.kD = slot0Config.pidConfig.kD;
        slot0.kS = slot0Config.ffConfig.kS;
        slot0.kG = slot0Config.ffConfig.kG;
        slot0.kV = slot0Config.ffConfig.kV;
        slot0.kA = slot0Config.ffConfig.kA;
    }
    protected void mapSlot1Config(Slot1Configs slot1, ControlConfig slot1Config) {
        slot1.kP = slot1Config.pidConfig.kP;
        slot1.kI = slot1Config.pidConfig.kI;
        slot1.kD = slot1Config.pidConfig.kD;
        slot1.kS = slot1Config.ffConfig.kS;
        slot1.kG = slot1Config.ffConfig.kG;
        slot1.kV = slot1Config.ffConfig.kV;
        slot1.kA = slot1Config.ffConfig.kA;
    }
    protected void mapSlot2Config(Slot2Configs slot2, ControlConfig slot2Config) {
        slot2.kP = slot2Config.pidConfig.kP;
        slot2.kI = slot2Config.pidConfig.kI;
        slot2.kD = slot2Config.pidConfig.kD;
        slot2.kS = slot2Config.ffConfig.kS;
        slot2.kG = slot2Config.ffConfig.kG;
        slot2.kV = slot2Config.ffConfig.kV;
        slot2.kA = slot2Config.ffConfig.kA;
    }
	protected void updateClosedLoopGeneralConfig(ClosedLoopGeneralConfigs closedLoopGeneral, ClosedLoopGeneralConfig closedLoopGeneralConfig) {
        closedLoopGeneral.ContinuousWrap = closedLoopGeneralConfig.continuousWrap;
    }
    protected void updateMotionMagicConfig(MotionMagicConfigs motionMagic, MotionMagicConfig motionMagicConfig) {
        motionMagic.MotionMagicCruiseVelocity = motionMagicConfig.cruiseVelocity.in(RotationsPerSecond);
        motionMagic.MotionMagicAcceleration = motionMagicConfig.acceleration.in(RotationsPerSecondPerSecond);
        motionMagic.MotionMagicJerk = motionMagicConfig.jerkRotationsPerSecCubed;
		motionMagic.MotionMagicExpo_kV = motionMagicConfig.expoKV;
		motionMagic.MotionMagicExpo_kA = motionMagicConfig.expoKA;
    }
    protected void updateCurrentLimitsConfig(CurrentLimitsConfigs currentLimits, CurrentLimitsConfig currentLimitsConfig) {
        currentLimits.StatorCurrentLimitEnable = currentLimitsConfig.statorCurrentLimitEnabled;
        currentLimits.StatorCurrentLimit = currentLimitsConfig.statorCurrentLimit.in(Amps);
        currentLimits.SupplyCurrentLimitEnable = currentLimitsConfig.supplyCurrentLimitEnabled;
        currentLimits.SupplyCurrentLimit = currentLimitsConfig.supplyCurrentLimit.in(Amps);
    }

	/*===================== Classes =====================*/

	public static class TalonIOConfigRequests<TalonConfigType extends TalonConfig> extends MotorIO.MotorIOConfigRequests<TalonConfigType> {
		protected boolean softLimitConfigChanged;
		protected boolean motorOutputConfigChanged;
		protected boolean slot0ConfigChanged;
		protected boolean slot1ConfigChanged;
		protected boolean slot2ConfigChanged;
		protected boolean closedLoopGeneralConfigChanged;
		protected boolean motionMagicConfigChanged;
		protected boolean currentLimitsConfigChanged;

		public void setSoftLimitConfig(SoftLimitConfig softLimitConfig) {
			configChanged |= softLimitConfigChanged = !config.softLimitConfig.equals(softLimitConfig);
			config.softLimitConfig = softLimitConfig;
		}
		public void setMotorOutputConfig(MotorOutputConfig motorOutputConfig) {
			configChanged |= motorOutputConfigChanged = !config.motorOutputConfig.equals(motorOutputConfig);
			config.motorOutputConfig = motorOutputConfig;
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
		public void setClosedLoopGeneralConfig(ClosedLoopGeneralConfig closedLoopGeneralConfig) {
			configChanged |= closedLoopGeneralConfigChanged = !config.closedLoopGeneralConfig.equals(config.closedLoopGeneralConfig);
			config.closedLoopGeneralConfig = closedLoopGeneralConfig;
		}
		public void setMotionMagicConfig(MotionMagicConfig motionMagicConfig) {
			configChanged |= motionMagicConfigChanged = !config.motionMagicConfig.equals(motionMagicConfig);
			config.motionMagicConfig = motionMagicConfig;
		}
		public void setCurrentLimitsConfig(CurrentLimitsConfig currentLimitsConfig) {
			configChanged |= currentLimitsConfigChanged |= !config.currentLimitsConfig.equals(currentLimitsConfig);
			config.currentLimitsConfig = currentLimitsConfig;
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