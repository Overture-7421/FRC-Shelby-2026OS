package frc.robot.Subsystems.Hood;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.overture.lib.motorcontrollers.OverTalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
	protected OverTalonFX hoodMotor;
	private Angle target = Degrees.of(0.0);

	private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0)
			.withEnableFOC(true);

	public Hood() {
		hoodMotor = new OverTalonFX(HoodConstants.motorConfig(), HoodConstants.motorCanId,
				Constants.RobotConstants.rio);

	}

	public Command setPosition(Angle position) {
		return run(() -> {
			setMotor(position);
		}).until(() -> isFinished());
	}

	public void setMotor(Angle position) {
		// Check if the position is within the allowed range
		if (position.gt(HoodConstants.Max)) {
			target = HoodConstants.Max;
		} else if (position.lt(HoodConstants.Min)) {
			target = HoodConstants.Min;
		}
		target = position;
		hoodMotor.setControl(motionMagicRequest.withPosition(target));
	}

	public double getTarget() {
		return target.in(Degrees);
	}

	public double getPosition() {
		return hoodMotor.getPosition().getValue().in(Degrees);
	}

	public double getError() {
		return Math.abs(getTarget() - getPosition());
	}

	private boolean isFinished() {
		return (getError() < HoodConstants.Control.AcceptedError.in(Degrees));
	}

	public void updateTelemetry() {
		SmartDashboard.putNumber("Subsystems/Hood/Position", getPosition());
		SmartDashboard.putNumber("Subsystems/Hood/Target", getTarget());
		SmartDashboard.putNumber("Subsystems/Hood/Error", getError());
		SmartDashboard.putBoolean("Subsystems/Hood/AtTarget", isFinished());
	}

	@Override
	public void periodic() {

	}

}
