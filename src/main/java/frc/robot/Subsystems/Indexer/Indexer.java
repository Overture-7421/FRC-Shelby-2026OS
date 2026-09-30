package frc.robot.Subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import frc.robot.Subsystems.Indexer.IndexerConstants;


public class Indexer extends SubsystemBase{

    protected OverTalonFX indexerMotorLead;
    protected OverTalonFX indexerMotor2;
    protected OverTalonFX indexerMotor3;
    protected OverTalonFX indexerMotor4;

    private double target;    

    private VoltageOut voltageRequest = new VoltageOut(0.0);

    Indexer(){
        indexerMotorLead = new OverTalonFX(IndexerConstants.motorConfig(), IndexerConstants.leaderCanId, Constants.canbus);    
        indexerMotor2 = new OverTalonFX(IndexerConstants.motorConfig(), IndexerConstants.MotorCanId2, Constants.canbus);
        indexerMotor3 = new OverTalonFX(IndexerConstants.motorConfig(), IndexerConstants.MotorCanId3, Constants.canbus);
        indexerMotor4 = new OverTalonFX(IndexerConstants.motorConfig(), IndexerConstants.MotorCanId4, Constants.canbus);
        
        indexerMotor2.setFollow(IndexerConstants.leaderCanId, false);
        indexerMotor3.setFollow(IndexerConstants.leaderCanId, false);    
        indexerMotor4.setFollow(IndexerConstants.leaderCanId, false);
    }
    
    private void setTarget(double targetSetter){
    target = targetSetter;
    }

    public Command setVoltage(Voltage volts){
        return 
            runOnce(()-> setTarget(volts.baseUnitMagnitude()))
            .andThen(runOnce(() -> indexerMotorLead.setControl(voltageRequest.withOutput(getTarget()))));
    }

    public double getTarget(){
        return target;
    }

    public void updateTelemetry(){
        SmartDashboard.putNumber("Subsystems/Indexer/Velocity", indexerMotorLead.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Subsystems/Indexer/Target", getTarget());
    }

}
