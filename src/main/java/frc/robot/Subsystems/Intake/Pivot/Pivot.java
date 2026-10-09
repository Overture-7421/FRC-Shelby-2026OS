package frc.robot.Subsystems.Intake.Pivot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.overture.lib.motorcontrollers.OverTalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
	protected OverTalonFX pivotMotor;
	protected OverTalonFX pivotCompressMotor;
	protected CANcoder pivotCC;

	private Angle target = Degrees.of(0.0);

	private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0)
			.withEnableFOC(true);

	public Pivot() {
		pivotCompressMotor = new OverTalonFX(
			PivotConstants.motorCompressConfig(),
			PivotConstants.motorCanId,
			Constants.RobotConstants.rio);
		pivotMotor = new OverTalonFX(
			PivotConstants.motorConfig(), 
			PivotConstants.motorCanId,
			Constants.RobotConstants.rio);
		pivotCC = new CANcoder(
			PivotConstants.CCCanId, 
			Constants.RobotConstants.rio);
		pivotCC.getConfigurator().apply(PivotConstants.CCConfig());

	}

	public Command setPosition(Angle position) {
		return run(() -> {
			setMotor(position);
		}).until(() -> isFinished());
	}

	public Command compress(){
		return run(() -> {
			target = PivotConstants.States.Closed;
			pivotCompressMotor.setControl(motionMagicRequest.withPosition(target));
		}).until(() -> isFinished());
	}

	public void setMotor(Angle position) {
		target = position;
		pivotMotor.setControl(motionMagicRequest.withPosition(target));
	}

	public double getTarget() {
		return target.in(Degrees);
	}

	public double getPosition() {
		return pivotMotor.getPosition().getValue().in(Degrees);
	}

	public double getError() {
		return Math.abs(getTarget() - getPosition());
	}

	private boolean isFinished() {
		return (getError() < PivotConstants.Control.AcceptedError.in(Degrees));
	}

	public void updateTelemetry() {
		SmartDashboard.putNumber("Subsystems/Pivot/Position", getPosition());
		SmartDashboard.putNumber("Subsystems/Pivot/Target", getTarget());
		SmartDashboard.putNumber("Subsystems/Pivot/Error", getError());
		SmartDashboard.putBoolean("Subsystems/Pivot/AtTarget", isFinished());
	}

	@Override
	public void periodic() {

	}

}
