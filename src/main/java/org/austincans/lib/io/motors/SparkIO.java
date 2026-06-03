package org.austincans.lib.io.motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.austincans.lib.io.motors.MotorConfig.ControlConfig;
import org.austincans.lib.io.motors.MotorConfig.FFConfig;
import org.austincans.lib.io.motors.MotorConfig.PIDConfig;
import org.austincans.lib.io.motors.MotorConfig.SoftLimitConfig;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/*
 * To configure while deployed, use kNoResetSafeParameters and kNoPersistParameters 
 * so that you can configure only specific values while keeping existing ones.
 */
public class SparkIO<MotorType extends SparkBase, VendorConfigType extends SparkBaseConfig> extends BaseMotorIO<MotorIO.MotorIOInputs, SparkConfig> {
    private static final ClosedLoopSlot[] slots = ClosedLoopSlot.values();

    protected final MotorType motor;
    protected final RelativeEncoder encoder;
    protected final SparkClosedLoopController controller;

    protected final VendorConfigType vendorConfig;

    protected SparkIO(MotorType motor, VendorConfigType vendorConfig, SparkConfig config) {
        super(config);

        this.motor = motor;
        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();

        /* Config */

        this.vendorConfig = vendorConfig;

        vendorConfig.inverted(config.inverted);
        vendorConfig.smartCurrentLimit((int) config.smartCurrentStallLimit.in(Amps));
        vendorConfig.idleMode(config.neutralMode == NeutralMode.BRAKE ? IdleMode.kBrake : IdleMode.kCoast);

        final SoftLimitConfig softLimitConfig = config.softLimitConfig;
        vendorConfig.softLimit.forwardSoftLimitEnabled(softLimitConfig.forwardSoftLimitEnabled);
        vendorConfig.softLimit.forwardSoftLimit(softLimitConfig.forwardSoftLimit.in(Rotations));
        vendorConfig.softLimit.reverseSoftLimitEnabled(softLimitConfig.reverseSoftLimitEnabled);
        vendorConfig.softLimit.reverseSoftLimit(softLimitConfig.reverseSoftLimit.in(Rotations));

        configureClosedLoopSlot(vendorConfig, config.slot0Config, ClosedLoopSlot.kSlot0, config);
        configureClosedLoopSlot(vendorConfig, config.slot1Config, ClosedLoopSlot.kSlot1, config);
        configureClosedLoopSlot(vendorConfig, config.slot2Config, ClosedLoopSlot.kSlot2, config);
        configureClosedLoopSlot(vendorConfig, config.slot3Config, ClosedLoopSlot.kSlot3, config);

        if (config.isFollower) {
            vendorConfig.follow(config.leaderCanId, config.invertFollowerOutput);
        }
        
        motor.configure(vendorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureClosedLoopSlot(VendorConfigType vendorConfig, ControlConfig controlConfig, ClosedLoopSlot slot, SparkConfig config) {
        PIDConfig pid = controlConfig.pidConfig;
        FFConfig ff = controlConfig.ffConfig;

        ClosedLoopConfig closedLoop = vendorConfig.closedLoop;
        closedLoop.pid(pid.kP, pid.kI, pid.kD, slot);
        closedLoop.feedForward.svag(ff.kS, ff.kV, ff.kA, ff.kG, slot);
        closedLoop.allowedClosedLoopError(controlConfig.allowedMechanismClosedLoopError.in(Rotations), slot);

        closedLoop.maxMotion.cruiseVelocity(config.maxSpeed.in(RPM), slot);
        closedLoop.maxMotion.maxAcceleration(config.maxAcceleration.in(RPM.per(Second)), slot);
    }

    @Override
    protected void setVoltage(Voltage voltage) {
        controller.setSetpoint(voltage.in(Volts), ControlType.kVoltage);
    }

    @Override
    protected void setVelocity(AngularVelocity velocity, ControlSlot slot) {
        controller.setSetpoint(velocity.in(RPM), ControlType.kVelocity, slots[slot.getIndex()]);
    }

    @Override
    protected void setPosition(Angle position, ControlSlot slot) {
        controller.setSetpoint(position.in(Rotations), ControlType.kPosition, slots[slot.getIndex()]);
    }

    @Override
    protected Angle getPosition() {
        return Rotations.of(encoder.getPosition());
    }

    @Override
    protected AngularVelocity getVelocity() {
        return RPM.of(encoder.getVelocity());
    }

    @Override
    protected Voltage getVoltage() {
        return Volts.of(motor.getBusVoltage());
    }

    @Override
    protected Current getStatorCurrent() {
        return Amps.of(motor.getOutputCurrent());
    }

    @Override
    protected Temperature getTemperature() {
        return Celsius.of(motor.getMotorTemperature());
    }

    @Override
    public void updateGains(ControlConfig config, ControlSlot slot) {
        configureClosedLoopSlot(vendorConfig, config, slots[slot.getIndex()], this.config);
        motor.configureAsync(vendorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /** TODO: This should probably be updated as part of outputs, or have a different method for updating configs */
    public void setEncoderPosition(double positionRotations) {
        encoder.setPosition(positionRotations);
    }

    public static enum SparkControlSlot implements MotorIO.ControlSlot {
        SLOT_0, 
        SLOT_1, 
        SLOT_2, 
        SLOT_3;

        public int getIndex() {
            return this.ordinal();
        }
    }

    @Override
    public void initializeOutputs(MotorIOOutputs outputs) {
        outputs.setControlSlot(config.initialSlot);
    }

    public enum SparkVelocityControlType implements VelocityControlType {

    }
}
