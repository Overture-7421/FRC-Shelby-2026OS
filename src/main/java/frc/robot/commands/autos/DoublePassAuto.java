// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Chassis.BLinePaths;

public class DoublePassAuto {
	/**
	 * @param mirrored false runs the path as drawn, along the right wall on blue;
	 *                 true mirrors it to the left wall
	 */
	public static Command DoublePass(BLinePaths paths, AutoCommands autos, boolean mirrored) {
		// Only the first path resets the pose. The others start where the previous one
		// ended and follow from wherever the robot really is
		return new SequentialCommandGroup(
				autos.intakeAlong(paths.resetAndFollow("firstpass", mirrored)),
				autos.hubShot(3.0),
				autos.intakeAlong(paths.follow("secondpass", mirrored)),
				autos.hubShot(3.0),
				autos.intakeAlong(paths.follow("ThirdPass", mirrored)));
	}
}
