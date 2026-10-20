package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.overture.lib.motorcontrollers.OverTalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
	protected OverTalonFX shooterMotorLead;
	protected OverTalonFX shooterMotor2;
	protected OverTalonFX shooterMotor3;
	protected OverTalonFX shooterMotor4;

	private AngularVelocity target = RotationsPerSecond.of(0.0);

	private VoltageOut voltageRequest = new VoltageOut(0.0);
	private MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0.0)
			.withEnableFOC(true);


	private int multiplier = 0;
	private final double manualOffset = 1.0;

	public Shooter() {
		shooterMotorLead = new OverTalonFX(ShooterConstants.motorConfig(), ShooterConstants.leaderCanId,
				Constants.RobotConstants.rio);
		shooterMotor2 = new OverTalonFX(ShooterConstants.motorConfig(), ShooterConstants.MotorCanId2, Constants.RobotConstants.rio);
		shooterMotor3 = new OverTalonFX(ShooterConstants.motorConfig(), ShooterConstants.MotorCanId3, Constants.RobotConstants.rio);
		shooterMotor4 = new OverTalonFX(ShooterConstants.motorConfig(), ShooterConstants.MotorCanId4, Constants.RobotConstants.rio);


		// One linkage for all four. The right motors are mounted mirrored, they follow
		// opposed or they fight the shaft (bottom right read negative on the robot)
		shooterMotor2.setFollow(ShooterConstants.leaderCanId, false);
		shooterMotor3.setFollow(ShooterConstants.leaderCanId, true);
		shooterMotor4.setFollow(ShooterConstants.leaderCanId, true);

	}

	public Command setVoltage(Voltage volts) {
		return runOnce(() -> shooterMotorLead.setControl(voltageRequest.withOutput(volts)));
	}

	public Command setVelocity(AngularVelocity rps) {
		return run(() -> {
			setMotor(rps);
		}).until(() -> isAtTarget());
	}

	public Command trackVelocity(Supplier<AngularVelocity> rps) {
		return run(() -> {
			setMotor(rps.get());
		});
	}

	public void setMotor(AngularVelocity rps) {
		target = rps.plus(RevolutionsPerSecond.of(manualOffset * multiplier));

		shooterMotorLead.setControl(velocityRequest.withVelocity(target));
	}

	// Lets the wheel coast down. Called when a launch ends, the shooter has no
	// default command that would do it
	public void stop() {
		target = RotationsPerSecond.of(0.0);
		shooterMotorLead.setControl(voltageRequest.withOutput(ShooterConstants.OffVoltage));
	}

	public double getTarget() {
		return target.in(RotationsPerSecond);
	}

	public double getVelocity() {
		return shooterMotorLead.getVelocity().getValueAsDouble();
	}

	public boolean isAtTarget() {
		return (Math.abs(getTarget() - getVelocity()) < ShooterConstants.Control.AcceptedError.in(RotationsPerSecond));
	}

	public boolean isNearTarget() {
		return (Math.abs(getTarget() - getVelocity()) < ShooterConstants.Control.FeedingError.in(RotationsPerSecond));
	}

	private void addOffset(int addedOffset){
		multiplier = multiplier + addedOffset ;
	}

	public Command AddOffset(int addedOffset){
		return runOnce(() -> {
			addOffset(addedOffset);
		});
	}

	public void updateTelemetry() {
		SmartDashboard.putNumber("Subsystems/Shooter/Velocity", getVelocity());
		SmartDashboard.putNumber("Subsystems/Shooter/Target", getTarget());
	}

	@Override
	public void periodic() {

	}

}
