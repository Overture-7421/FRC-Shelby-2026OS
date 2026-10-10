package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Hood.HoodConstants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import frc.robot.RobotContainer;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

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


public class HubOrPass {

    HubOrPass(){
    }
    
    public static Command hubOrPass(LaunchModes mode, Hood hood) {

        switch(mode){

            case PASS:
                return new ParallelCommandGroup(
                    //aqui va el align,
                    hood.setPosition(null)

                );
            
            
            case HUB:
                return new ParallelCommandGroup(
                    //aqui va el align,
                    hood.setPosition(null)

                );
            
            case IDLE:
                return new ParallelCommandGroup(
                    //aqui va el align,
                    hood.setPosition(null)

                );

            default:
                return null;
                
        }        

    }
    
    public enum LaunchModes {
		PASS,
		HUB,
        IDLE
	}
    
}

