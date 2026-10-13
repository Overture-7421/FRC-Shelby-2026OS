package frc.robot.Subsystems.Intake.Rollers;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VoltageOut;
import com.overture.lib.motorcontrollers.OverTalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Roller extends SubsystemBase{
    
    protected OverTalonFX rollerLeadMotor;
    protected OverTalonFX rollerSlaveMotor;
    private Voltage target = Volts.of(0.0);
    
    protected final VoltageOut voltageRequest = new VoltageOut(0.0);

    public Roller(){
        rollerLeadMotor = new OverTalonFX(RollerConstants.motorConfig(), RollerConstants.motorCanId, Constants.RobotConstants.rio);
        rollerSlaveMotor = new OverTalonFX(RollerConstants.motorConfig(), RollerConstants.MotorCanId2, Constants.RobotConstants.rio);

        rollerSlaveMotor.setFollow(RollerConstants.motorCanId, true);
    }

    // Drives the motor straight away, for commands that move the rollers every loop
    public void setTarget(Voltage targetSetter){
        target = targetSetter;
        rollerLeadMotor.setControl(voltageRequest.withOutput(target));
    }

    public double getTarget(){
        return target.baseUnitMagnitude();
    }

    public Command setVoltage(Voltage volts){
        return runOnce(() -> {
            setTarget(volts);
        });
    }

	public double getVoltage() {
		return rollerLeadMotor.getMotorVoltage().getValueAsDouble();
	}

    public double getVelocity(){
        return rollerLeadMotor.getVelocity().getValueAsDouble();
    }

	public void updateTelemetry() {
		SmartDashboard.putNumber("Subsystems/Roller/Voltage", getVoltage());
		SmartDashboard.putNumber("Subsystems/Roller/Target", getTarget());
	}

	@Override
	public void periodic() {
	}

}
