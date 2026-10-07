// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Hood.Hood;

import frc.robot.Subsystems.Intake.Pivot.Pivot;
import frc.robot.Subsystems.Intake.Rollers.Roller;;


public class RobotContainer {
	public static Hood hood = new Hood();
	public static Pivot pivot = new Pivot();
	public static Roller roller = new Roller();


	public RobotContainer() {}
	
	

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
