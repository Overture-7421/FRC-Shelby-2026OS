package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.overture.lib.motorcontrollers.OverTalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

    
    
    
    

public class Shooter extends SubsystemBase{
    protected OverTalonFX shooterMotorLead;
    protected OverTalonFX shooterMotor2;
    protected OverTalonFX shooterMotor3;
    protected OverTalonFX shooterMotor4;

    private double target;

    private VoltageOut voltageRequest = new VoltageOut(0.0);
    private MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0.0)
			.withEnableFOC(true);

    Shooter() {
    shooterMotorLead = new OverTalonFX(ShooterConstants.motorConfig(), ShooterConstants.leaderCanId, Constants.canbus);    
    shooterMotor2 = new OverTalonFX(ShooterConstants.motorConfig(), ShooterConstants.MotorCanId2, Constants.canbus);
    shooterMotor3 = new OverTalonFX(ShooterConstants.motorConfig(), ShooterConstants.MotorCanId3, Constants.canbus);
    shooterMotor4 = new OverTalonFX(ShooterConstants.motorConfig(), ShooterConstants.MotorCanId4, Constants.canbus);
    
    shooterMotor2.setFollow(ShooterConstants.leaderCanId, false);
    shooterMotor3.setFollow(ShooterConstants.leaderCanId, false);    
    shooterMotor4.setFollow(ShooterConstants.leaderCanId, false);


    }

    private void setTarget(double targetSetter){
    target = targetSetter;
    }

    public Command setVoltage(Voltage volts){
        return 
            runOnce(()-> setTarget(volts.baseUnitMagnitude()))
            .andThen(runOnce(() -> shooterMotorLead.setControl(voltageRequest.withOutput(getTarget()))));
    }

    public Command setVelocity(AngularVelocity rps){
        return
            runOnce(() -> setTarget(rps.baseUnitMagnitude()))
            .andThen(runOnce(() -> shooterMotorLead.setControl(velocityRequest.withVelocity(getTarget()))))
            .until(() -> isFinished());
    }

    public double getTarget(){
        return target;
    }

    public double getVelocity(){
        return shooterMotorLead.getVelocity().getValueAsDouble();
    }

    private boolean isFinished(){
        return (Math.abs( getTarget() - getVelocity()) < ShooterConstants.Control.AcceptedError);
    }
  
    public void updateTelemetry(){
        SmartDashboard.putNumber("Subsystems/Shooter/Velocity", getVelocity());
        SmartDashboard.putNumber("Subsystems/Shooter/Target", getTarget());
    }

    @Override
    public void periodic(){

    }

}
