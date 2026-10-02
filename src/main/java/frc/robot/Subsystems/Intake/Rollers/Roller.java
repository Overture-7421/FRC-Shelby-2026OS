package frc.robot.Subsystems.Intake.Rollers;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VoltageOut;
import com.overture.lib.motorcontrollers.OverTalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Roller extends SubsystemBase {
	protected OverTalonFX rollerMotor;

	private Voltage target = Volts.of(0.0);

	private VoltageOut voltageRequest = new VoltageOut(0.0);

	public Roller() {
		rollerMotor = new OverTalonFX(RollerConstants.motorConfig(), RollerConstants.motorCanId,
				Constants.RobotConstants.rio);
	}

	public Command setVoltage(Voltage volts) {
		return runOnce(() -> {
			setMotor(volts);
		});
	}

	public void setMotor(Voltage volts) {
		target = volts;
		rollerMotor.setControl(voltageRequest.withOutput(target));
	}

	public double getTarget() {
		return target.in(Volts);
	}

	public double getVoltage() {
		return rollerMotor.getMotorVoltage().getValueAsDouble();
	}

	public void updateTelemetry() {
		SmartDashboard.putNumber("Subsystems/Roller/Voltage", getVoltage());
		SmartDashboard.putNumber("Subsystems/Roller/Target", getTarget());
	}

	@Override
	public void periodic() {
	}

}
