package frc.robot.Subsystems.Indexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VoltageOut;
import com.overture.lib.motorcontrollers.OverTalonFX;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;


public class Indexer extends SubsystemBase{

    protected OverTalonFX indexerMotorLead;
    protected OverTalonFX indexerMotor2;
    protected OverTalonFX indexerMotor3;
    protected OverTalonFX indexerMotor4;

    protected CANrange shooterCanRange;
    protected CANrange hopperCanRange;

	// The hopper sensor is debounced so a single fuel rolling by does not count as
	// a full hopper. The shooter sensor is read straight, a shot has to react fast
	protected boolean hopperFull;
	protected Debouncer debouncer;

	private Voltage target = Volts.of(0.0);

	private VoltageOut voltageRequest = new VoltageOut(0.0);

    public Indexer() {
        indexerMotorLead = new OverTalonFX(IndexerConstants.motorConfig(), IndexerConstants.leaderCanId, Constants.RobotConstants.rio);    
        indexerMotor2 = new OverTalonFX(IndexerConstants.motorConfig(), IndexerConstants.MotorCanId2, Constants.RobotConstants.rio);
        indexerMotor3 = new OverTalonFX(IndexerConstants.motorConfig(), IndexerConstants.MotorCanId3, Constants.RobotConstants.rio);
        indexerMotor4 = new OverTalonFX(IndexerConstants.motorConfig(), IndexerConstants.MotorCanId4, Constants.RobotConstants.rio);

        shooterCanRange = new CANrange(IndexerConstants.shooterCanRangeId, Constants.RobotConstants.rio);
        shooterCanRange.getConfigurator().apply(IndexerConstants.rangeConfig());
        hopperCanRange = new CANrange(IndexerConstants.hopperCanRangeId, Constants.RobotConstants.rio);
        hopperCanRange.getConfigurator().apply(IndexerConstants.rangeConfig());
		debouncer = new Debouncer(IndexerConstants.fuelDebouncingTime, DebounceType.kRising);
        
        indexerMotor2.setFollow(IndexerConstants.leaderCanId, false);
        indexerMotor3.setFollow(IndexerConstants.leaderCanId, false);    
        indexerMotor4.setFollow(IndexerConstants.leaderCanId, false);
    }

	public Command setVoltage(Voltage volts) {
		return runOnce(() -> {
			setTarget(volts);
		});
	}

	// What the indexer does when nobody is shooting. It never finishes, it is the
	// default command
	public Command preloadShooter() {
		return run(() -> {
			setMotorPreload();
		});
	}

	public void setTarget(Voltage volts) {
		target = volts;
		indexerMotorLead.setControl(voltageRequest.withOutput(target));
	}

	// Creeps the fuel towards the shooter while the hopper has fuel and nothing is
	// waiting at the shooter yet, so the first shot leaves without delay
	public void setMotorPreload() {
		if (isHopperFull() && !isShooterFull()) {
			setTarget(IndexerConstants.preloadVoltage);
		} else {
			setTarget(IndexerConstants.OffVoltage);
		}
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
		SmartDashboard.putBoolean("Subsystems/Indexer/HopperFull", isHopperFull());
		SmartDashboard.putBoolean("Subsystems/Indexer/ShooterFull", isShooterFull());
	}

	private boolean isFuelInHopper(){
		return hopperCanRange.getIsDetected().getValue();
	}

	private boolean isFuelInShooter(){
		return shooterCanRange.getIsDetected().getValue();
	}
	
	public boolean isHopperFull(){
		return hopperFull;
	}

	public boolean isShooterFull(){
		return isFuelInShooter();
	}

	@Override
	public void periodic() {
		hopperFull = debouncer.calculate(isFuelInHopper());
	}

}
