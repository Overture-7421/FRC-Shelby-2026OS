package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Intake.Pivot.Pivot;
import frc.robot.Subsystems.Intake.Pivot.PivotConstants;
import frc.robot.Subsystems.Intake.Rollers.Roller;
import frc.robot.Subsystems.Intake.Rollers.RollerConstants;
import frc.robot.RobotContainer;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public class CloseAndCompress {
    public CloseAndCompress() {
    }

    public static Command closeAndCompres(){
         return new ParallelCommandGroup(
            RobotContainer.pivot.compress(),
            RobotContainer.roller.setVoltage(RollerConstants.CompressingVoltage)
            );
    }

    
}
