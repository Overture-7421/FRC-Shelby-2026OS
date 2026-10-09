package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Indexer.IndexerConstants;
import frc.robot.Subsystems.Intake.Pivot.Pivot;
import frc.robot.Subsystems.Intake.Pivot.PivotConstants;
import frc.robot.Subsystems.Intake.Rollers.Roller;
import frc.robot.Subsystems.Intake.Rollers.RollerConstants;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import frc.robot.RobotContainer;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public class PreloadShooter {
    public PreloadShooter() {
    }

   

    public static Command preloadShooter(){

        if (RobotContainer.indexer.isFuelInShooter() && RobotContainer.indexer.isFuelInHopper()){
            return new ParallelCommandGroup(
                RobotContainer.indexer.setVoltage(IndexerConstants.OffVoltage),
                RobotContainer.shooter.setVoltage(ShooterConstants.OffVoltage)
            );
            
        } else if(RobotContainer.indexer.isFuelInShooter() && !RobotContainer.indexer.isFuelInHopper()){
            return new ParallelCommandGroup(
                RobotContainer.indexer.setVoltage(IndexerConstants.preloadVoltage),
                RobotContainer.shooter.setVoltage(ShooterConstants.OffVoltage)
            );


        } else {
            return new ParallelCommandGroup(
                RobotContainer.indexer.setVoltage(IndexerConstants.preloadVoltage),
                RobotContainer.shooter.setVoltage(ShooterConstants.preloadVoltage)
            );
        }
        
    }
}
