package org.austincans.frc2158.subsystems.hood;
import org.austincans.frc2158.math.Angle;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;

public class HoodIOTalonFXS implements HoodIO{
    private TalonFXS hoodMotor = new TalonFXS(HoodConstants.talonMotorID);
    private DutyCycleOut dc = new DutyCycleOut(0);
    TalonFXSConfiguration configs = new TalonFXSConfiguration();

    public HoodIOTalonFXS(){
        configs.Slot0.kP = 1;
        configs.Slot0.kI = 0;
        configs.Slot0.kD = 10;
        configs.Slot0.kV = 2;
    }

    public void runHoodMotor(double speed){
        hoodMotor.set(speed);
    }

    @Override
    public Angle getAngle() {
        return Angle.fromDegrees(hoodMotor.getPosition().getValue().in(null));
    }

    public void setHoodAngle(Angle setPointDegrees){
        hoodMotor.setControl(dc.withOutput(Units.degreesToRotations(setPointDegrees.toDegrees())));
    }

    @Override
    public void setAngleRotation2d(Rotation2d angle) {
        hoodMotor.setControl(dc.withOutput(Units.degreesToRotations(angle.getDegrees())));
    }

    @Override
    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromRadians(Units.rotationsToRadians(hoodMotor.getPosition().getValue().in(null)));
    }

    @Override
    public boolean atAngle(Angle setPoint){
        if (getAngle() == setPoint){
            return true;
        }
        else{
            return false;
        }
    }

    public void updateInputs(hoodIOInputs inputs){
        inputs.hoodAngleDegrees = getAngle();
        inputs.temperature = hoodMotor.getDeviceTemp().getValueAsDouble();
        inputs.motorPosition = hoodMotor.getPosition().getValue().in(null);
    }

}
