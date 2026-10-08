package frc.robot.Subsystems.Hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.overture.lib.motorcontrollers.OverTalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
	protected OverTalonFX hoodMotor;
	private Angle target = Degrees.of(0.0);

	private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0)
			.withEnableFOC(true);
	private VoltageOut voltageRequest = new VoltageOut(0.0);

	private boolean homed = false;
	private final Debouncer touchingDebouncer = new Debouncer(HoodConstants.Control.HomingSettleTime,
			DebounceType.kRising);

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
		if (position.gt(HoodConstants.States.Max)) {
			target = HoodConstants.States.Max;
		} else if (position.lt(HoodConstants.States.Min)) {
			target = HoodConstants.States.Min;
		} else {
			target = position;
		}
		hoodMotor.setControl(motionMagicRequest.withPosition(target));
	}

	private void setVoltage(Voltage volts) {
		hoodMotor.setControl(voltageRequest.withOutput(volts));
	}

	public Command setVoltageCommand(Voltage volts){
		return runOnce(() -> setVoltage(volts));
	}

	public double getTarget() {
		return target.in(Degrees);
	}

	public double getAmps() {
		return hoodMotor.getStatorCurrent().getValueAsDouble();
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

	private boolean isHome() {
		return touchingDebouncer.calculate(getAmps() > HoodConstants.Control.TouchingCurrentThreshold.in(Amps));
	}

	public Command HoodHoming() {
		return runOnce(() -> {
			homed = false;
			touchingDebouncer.calculate(false);
		}).andThen(run(() -> {
			setVoltage(Volts.of(HoodConstants.Control.HomingVoltage.in(Volts)));
		}).until(() -> isHome())).finallyDo((interrupted) -> {
			setVoltage(Volts.of(0.0));
			if (!interrupted) {
				hoodMotor.setPosition(HoodConstants.Control.HomedPosition);
				homed = true;
			}
		});
	}

	public void updateTelemetry() {
		SmartDashboard.putNumber("Subsystems/Hood/Position", getPosition());
		SmartDashboard.putNumber("Subsystems/Hood/Target", getTarget());
		SmartDashboard.putNumber("Subsystems/Hood/Error", getError());
		SmartDashboard.putBoolean("Subsystems/Hood/AtTarget", isFinished());

		SmartDashboard.putNumber("Subsystems/Hood/Amps", getAmps());
		SmartDashboard.putBoolean("Subsystems/Hood/Homed", homed);
	}

	@Override
	public void periodic() {

	}

}
