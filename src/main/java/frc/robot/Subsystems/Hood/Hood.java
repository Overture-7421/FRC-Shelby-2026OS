package frc.robot.Subsystems.Hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.overture.lib.motorcontrollers.OverTalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
	protected OverTalonFX hoodMotor;
	private Angle target = Degrees.of(0.0);

	private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0)
			.withEnableFOC(true);
	private VoltageOut voltageRequest = new VoltageOut(0.0);

	public Hood() {
		hoodMotor = new OverTalonFX(HoodConstants.motorConfig(), HoodConstants.motorCanId, Constants.canbus);

	}

	public Command setPosition(Angle position) {
		return run(() -> {
			setMotor(position);
		}).until(() -> isFinished());
	}

	public void setMotor(Angle position) {
		// Check if the position is within the allowed range
		if (position.gt(HoodConstants.States.Max)) {
			target = HoodConstants.States.Max;
		} else if (position.lt(HoodConstants.States.Min)) {
			target = HoodConstants.States.Min;
		}
		target = position;
		hoodMotor.setControl(motionMagicRequest.withPosition(target));
	}

	private void setVoltage(Voltage volts){
		hoodMotor.setControl(voltageRequest.withOutput(volts));
	}

	public double getTarget() {
		return target.in(Degrees);
	}

	public double getAmps(){
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

	private boolean isHome(){
		return (getAmps() > HoodConstants.Control.TouchingCurrentTreshold.baseUnitMagnitude());
	}


	public Command HoodHoming(){
		return 
			runOnce(()-> setVoltage(Volts.of(1*HoodConstants.Control.DirectionOfHoming)))
			.andThen(new WaitUntilCommand(() -> isHome()))
			.andThen(() -> setVoltage(Volts.of(0.0)))
			.andThen(() -> {HoodConstants.Control.OffSet = Degrees.of(hoodMotor.getPosition().getValueAsDouble());});
			
			/*
			* para hacerlo por velocidad checa en esta parte del code de 2910:
			* src\main\java\org\frc2910\robot\subsystems\base\servo\ServoMotorSubsystem.java
			* funcion home
			*/
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
