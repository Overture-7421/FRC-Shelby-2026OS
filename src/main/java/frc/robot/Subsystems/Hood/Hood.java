package frc.robot.Subsystems.Hood;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.overture.lib.motorcontrollers.OverTalonFX;
import frc.robot.Constants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.*;

public class Hood extends SubsystemBase{
    protected OverTalonFX hoodMotor;
    protected CANcoder hoodCC;
    private Angle target;

    protected final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    
     public Hood(){
        
        hoodMotor = new OverTalonFX(HoodConstants.motorConfig(), HoodConstants.motorCanId, Constants.canbus);
        hoodCC = new CANcoder(HoodConstants.CCCanId, Constants.canbus);
        hoodCC.getConfigurator().apply(HoodConstants.CCConfig());
    }

    public Angle getTarget(){
        return target;
    }

    public Angle getPosition(){
        return Degrees.of(hoodMotor.getPosition().getValueAsDouble());
    }

    private Angle getError(){
        Angle error = getTarget().minus(getPosition());
        return Degrees.of(Math.abs(error.baseUnitMagnitude()));
    }
    
    public void setTarget(Angle targetSetter){
        target = targetSetter;
    }

    public Command setPosition(Angle rotations){
        return  
            runOnce(() -> setTarget(rotations))
            .andThen(run(() -> hoodMotor.setControl(motionMagicRequest.withPosition(target))).until(() -> isFinished()));
    }

    public Command setClosed(){
        return setPosition(HoodConstants.States.Closed);
    }

    private boolean isFinished(){
        return getError().baseUnitMagnitude() < HoodConstants.Control.AcceptedError.baseUnitMagnitude();
    }
    
    public void updateTelemetry(){
        SmartDashboard.putNumber("Subsystems/Arm/Position", getPosition().baseUnitMagnitude());
        SmartDashboard.putNumber("Subsystems/Arm/Target", getTarget().baseUnitMagnitude());
        SmartDashboard.putNumber("Subsystems/Arm/Error", getError().baseUnitMagnitude());
        SmartDashboard.putBoolean("Subsystems/Arm/Position", isFinished());
    }

    
}
