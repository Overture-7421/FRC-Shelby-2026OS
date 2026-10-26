package frc.robot.Subsystems.Intake.Pivot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.overture.lib.motorcontrollers.OverTalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
	protected OverTalonFX pivotMotor;
	protected CANcoder pivotCC;

	private Angle target = Degrees.of(0.0);

	private double positionDegrees = 0.0;

	private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0)
			.withEnableFOC(true);

	private boolean slowLimits = false;

	public Pivot() {
		pivotMotor = new OverTalonFX(PivotConstants.motorConfig(), PivotConstants.motorCanId,
				Constants.RobotConstants.rio);
		pivotCC = new CANcoder(PivotConstants.CCCanId, Constants.RobotConstants.rio);
		pivotCC.getConfigurator().apply(PivotConstants.CCConfig());

	}

	public Command setPosition(Angle position) {
		return run(() -> {
			setMotor(position);
		}).until(() -> isAtTarget());
	}

	public Command setPositionSlow(Angle position) {
		return run(() -> {
			setMotorSlow(position);
		}).until(() -> isAtTarget());
	}

	public void setMotor(Angle position) {
		applyLimits(false);
		target = position;
		pivotMotor.setControl(motionMagicRequest.withPosition(target));
	}

	public void setMotorSlow(Angle position) {
		applyLimits(true);
		target = position;
		pivotMotor.setControl(motionMagicRequest.withPosition(target));
	}

	private void applyLimits(boolean slow) {
		if (slow == slowLimits) {
			return;
		}
		slowLimits = slow;
		double speedScale = slow ? PivotConstants.Control.CompressSpeedScale : 1.0;
		double accelScale = slow ? PivotConstants.Control.CompressAccelScale : 1.0;
		pivotMotor.getConfigurator().apply(new MotionMagicConfigs()
				.withMotionMagicCruiseVelocity(PivotConstants.Control.CruiseVelocity.times(speedScale))
				.withMotionMagicAcceleration(PivotConstants.Control.AccelerationLimit.times(accelScale))
				.withMotionMagicJerk(PivotConstants.Control.JerkLimit));
	}

	public double getTarget() {
		return target.in(Degrees);
	}

	public double getPosition() {
		return positionDegrees;
	}

	public double getError() {
		return Math.abs(getTarget() - getPosition());
	}

	public boolean isAtTarget() {
		return (getError() < PivotConstants.Control.AcceptedError.in(Degrees));
	}

	public void updateTelemetry() {
		// SmartDashboard.putNumber("Subsystems/Pivot/Position", getPosition());
		// SmartDashboard.putNumber("Subsystems/Pivot/Target", getTarget());
		// SmartDashboard.putNumber("Subsystems/Pivot/Error", getError());
		SmartDashboard.putBoolean("Subsystems/Pivot/AtTarget", isAtTarget());
	}

	@Override
	public void periodic() {
		positionDegrees = Units.rotationsToDegrees(pivotMotor.getPosition().getValueAsDouble());
	}

}
