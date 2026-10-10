package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Hood.HoodConstants;

// Everything that runs while the driver holds a launch button. The hood is held
// (not setPosition, which would end and hand the hood back to its default
// command mid-shot). The align and the shooter speed go in the same group later
public class HubOrPass {

    HubOrPass(){
    }
    
    public static Command hubOrPass(LaunchModes mode, Hood hood) {

        switch(mode){

            case PASS:
                return new ParallelCommandGroup(
                    //aqui va el align,
                    hood.holdPosition(HoodConstants.States.Pass)
                );
            
            case HUB:
            default:
                return new ParallelCommandGroup(
                    //aqui va el align,
                    hood.holdPosition(HoodConstants.States.Hub)
                );
        }        

    }
    
    public enum LaunchModes {
		PASS,
		HUB
	}
    
}
