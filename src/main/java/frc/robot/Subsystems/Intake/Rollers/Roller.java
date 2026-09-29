package frc.robot.Subsystems.Intake.Rollers;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import java.util.concurrent.CancellationException;
import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.overture.lib.motorcontrollers.OverTalonFX;
import com.overture.lib.robots.RobotConstants;
import com.overture.lib.sensors.CanCoderConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Roller extends SubsystemBase{
    
    protected OverTalonFX rollerMotor;
    private double target;
    
    protected final VoltageOut voltageRequest = new VoltageOut(0.0);

    public Roller(){
        rollerMotor = new OverTalonFX(RollerConstants.motorConfig(), RollerConstants.motorCanId, Constants.canbus);

    }

    public void setTarget(double targetSetter){
        target = targetSetter;
    }

    public double getTarget(){
        return target;
    }

    public Command setVoltage(double volts){
        return runOnce(() -> rollerMotor.setVoltage(volts));
    }

    public double getVelocity(){
        return rollerMotor.getVelocity().getValueAsDouble();
    }

    

    public void updateTelemetry(){
        SmartDashboard.putNumber("Subsystems/Shooter/Velocity", getVelocity());
        SmartDashboard.putNumber("Subsystems/Shooter/Target", getTarget());
    }

    @Override
    public void periodic(){

    }
}
