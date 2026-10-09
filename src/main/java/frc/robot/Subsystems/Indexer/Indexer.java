package frc.robot.Subsystems.Indexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VoltageOut;
import com.overture.lib.motorcontrollers.OverTalonFX;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.core.CoreCANrange;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterConstants;


public class Indexer extends SubsystemBase{

    protected OverTalonFX indexerMotorLead;
    protected OverTalonFX indexerMotor2;
    protected OverTalonFX indexerMotor3;
    protected OverTalonFX indexerMotor4;

    protected CANrange shooterCanRange;
    protected CANrange hopperCanRange;

	private Voltage target = Volts.of(0.0);

	private VoltageOut voltageRequest = new VoltageOut(0.0);

    public Indexer() {
        indexerMotorLead = new OverTalonFX(IndexerConstants.motorConfig(), IndexerConstants.leaderCanId, Constants.RobotConstants.rio);    
        indexerMotor2 = new OverTalonFX(IndexerConstants.motorConfig(), IndexerConstants.MotorCanId2, Constants.RobotConstants.rio);
        indexerMotor3 = new OverTalonFX(IndexerConstants.motorConfig(), IndexerConstants.MotorCanId3, Constants.RobotConstants.rio);
        indexerMotor4 = new OverTalonFX(IndexerConstants.motorConfig(), IndexerConstants.MotorCanId4, Constants.RobotConstants.rio);

        shooterCanRange = new CANrange(IndexerConstants.shooterCanRangeId, Constants.RobotConstants.rio);
        hopperCanRange = new CANrange(IndexerConstants.hopperCanRangeId, Constants.RobotConstants.rio);
        
        indexerMotor2.setFollow(IndexerConstants.leaderCanId, false);
        indexerMotor3.setFollow(IndexerConstants.leaderCanId, false);    
        indexerMotor4.setFollow(IndexerConstants.leaderCanId, false);
    }
    
    

	public Command setVoltage(Voltage volts) {
		return runOnce(() -> {
			setTarget(volts);
		});
	}

	public void setTarget(Voltage volts) {
		target = volts;
		indexerMotorLead.setControl(voltageRequest.withOutput(target));
	}

	public double getTarget() {
		return target.in(Volts);
	}

	public double getVoltage() {
		return indexerMotorLead.getMotorVoltage().getValueAsDouble();
	}

	public void updateTelemetry() {
		SmartDashboard.putNumber("Subsystems/Indexer/Voltage", getVoltage());
		SmartDashboard.putNumber("Subsystems/Indexer/Target", getTarget());
	}

	public Boolean isFuelInHopper(){
		return (shooterCanRange.getDistance().getValueAsDouble() < IndexerConstants.fuelInShooterTreshold);
	}

	public Boolean isFuelInShooter(){
		return (hopperCanRange.getDistance().getValueAsDouble() < IndexerConstants.fuelInHopperTreshold);
	}
	
	

	@Override
	public void periodic() {

	}

}
