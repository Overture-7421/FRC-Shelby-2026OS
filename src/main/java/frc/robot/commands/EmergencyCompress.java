package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.IndexerConstants;
import frc.robot.Subsystems.Intake.Pivot.Pivot;
import frc.robot.Subsystems.Intake.Pivot.PivotConstants;
import frc.robot.Subsystems.Intake.Rollers.Roller;
import frc.robot.Subsystems.Intake.Rollers.RollerConstants;


public class EmergencyCompress extends Command{

    private final Pivot pivot;
	private final Roller roller;

    public EmergencyCompress(Pivot pivot, Roller roller){
        this.pivot = pivot;
		this.roller = roller;

    }


    @Override
    public void execute(){
        pivot.setMotorSlow(PivotConstants.States.Closed);
        roller.setVoltage(RollerConstants.CompressingVoltage);
    }

    @Override
    public void end(boolean interrupted){
        pivot.setMotor(PivotConstants.States.Open);
        roller.setVoltage(RollerConstants.OffVoltage);
    }


}
