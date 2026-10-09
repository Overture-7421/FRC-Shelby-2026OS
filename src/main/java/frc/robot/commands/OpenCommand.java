package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.Intake.Pivot.Pivot;
import frc.robot.Subsystems.Intake.Pivot.PivotConstants;
import frc.robot.Subsystems.Intake.Rollers.Roller;
import frc.robot.Subsystems.Intake.Rollers.RollerConstants;
import frc.robot.RobotContainer;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

// import edu.wpi.first.wpilibj2.command.Command;

public class OpenCommand {

    public OpenCommand() {
    }

    public static Command openCommand(Pivot pivot, Roller roller){
         return new ParallelCommandGroup(
            pivot.setPosition(PivotConstants.States.Open),
            roller.setVoltage(RollerConstants.CompressingVoltage)
            );
    }

}
 



