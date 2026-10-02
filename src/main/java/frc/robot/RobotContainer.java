// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.overture.lib.gamepads.OverXboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Chassis.Chassis;
import frc.robot.commands.DriveCommand;

public class RobotContainer {

	// Subsystems
	public final Chassis chassis = new Chassis();

	// Controllers
	private final OverXboxController driver = new OverXboxController(0, 0.20, 0.2);

	public RobotContainer() {
		configureBindings();
	}

	private void configureBindings() {
		chassis.setDefaultCommand(new DriveCommand(chassis, driver));
		driver.back().onTrue(chassis.resetHeadingCommand());
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
