// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.overture.lib.gamepads.OverXboxController;
import com.overture.lib.robots.OverContainer;
import com.overture.lib.utils.UtilityFunctions;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Chassis.BLinePaths;
import frc.robot.Subsystems.Chassis.Chassis;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.commands.DriveCommand;

public class RobotContainer implements OverContainer {

	// Subsystems
	public final Chassis chassis = new Chassis();
	protected Hood hood = new Hood();

	// Controllers. There is no operator controller, the driver does everything
	private final OverXboxController driver = new OverXboxController(0, 0.20, 0.2);

	// Autonomous
	private final BLinePaths paths = new BLinePaths(chassis);
	private final SendableChooser<Command> autoChooser = new SendableChooser<>();

	public RobotContainer() {
		configDriverBindings();
		configOperatorBindings();
		configCharacterizationBindings();
		configureAutos();
	}

	@Override
	public void configDriverBindings() {
		chassis.setDefaultCommand(new DriveCommand(chassis, driver));

		driver.back().onTrue(Commands.runOnce(() -> {
			chassis.resetHeading(UtilityFunctions.isRedAlliance() ? 180.0 : 0.0);
		}));
	}

	@Override
	public void configOperatorBindings() {
		// No operator controller
	}

	@Override
	public void configCharacterizationBindings() {
		// No characterization controller
	}

	private void configureAutos() {
		autoChooser.setDefaultOption("None", Commands.none());

		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	@Override
	public void updateTelemetry() {
		chassis.shuffleboardPeriodic();

		SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
	}
}
