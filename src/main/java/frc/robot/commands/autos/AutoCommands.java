// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Chassis.Chassis;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Intake.Pivot.Pivot;
import frc.robot.Subsystems.Intake.Pivot.PivotConstants;
import frc.robot.Subsystems.Intake.Rollers.Roller;
import frc.robot.Subsystems.Intake.Rollers.RollerConstants;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.commands.LaunchCommand;
import frc.robot.commands.LaunchModes;
import frc.robot.lib.BLine.FollowPath;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * The pieces every auto is built from. Each auto class strings these together
 * with its paths, so the robot behaves the same way in every auto and a change
 * here reaches all of them.
 */
public final class AutoCommands {
	private final Shooter shooter;
	private final Hood hood;
	private final Chassis chassis;
	private final Indexer indexer;
	private final Pivot pivot;
	private final Roller roller;
	private final DoubleSupplier hubMulti;
	private final BooleanSupplier tagVisible;

	/**
	 * @param hubMulti   the hub shooter trim from the dashboard, read fresh each loop
	 * @param tagVisible whether any camera sees a tag, for the hub shot
	 */
	public AutoCommands(Shooter shooter, Hood hood, Chassis chassis, Indexer indexer, Pivot pivot, Roller roller,
			DoubleSupplier hubMulti, BooleanSupplier tagVisible) {
		this.shooter = shooter;
		this.hood = hood;
		this.chassis = chassis;
		this.indexer = indexer;
		this.pivot = pivot;
		this.roller = roller;
		this.hubMulti = hubMulti;
		this.tagVisible = tagVisible;

		FollowPath.registerEventTrigger("intakeOn", () -> roller.setTarget(RollerConstants.IntakingVoltage));
		FollowPath.registerEventTrigger("intakeOff", () -> roller.setTarget(RollerConstants.OffVoltage));
	}

	/**
	 * Aims at the hub and feeds whenever the shot is ready, the same launch the
	 * driver holds on the right trigger, for a fixed time. There is no sensor for
	 * an empty hopper, so the time is how long it takes to empty it.
	 *
	 * @param seconds how long to keep shooting
	 * @return the command
	 */
	public Command hubShot(double seconds) {
		return new LaunchCommand(shooter, hood, chassis, indexer, pivot, roller,
				LaunchModes.HUB, hubMulti, tagVisible, () -> false)
				.withTimeout(seconds);
	}

	/**
	 * Intake out and rollers in for as long as a path runs, then rollers off. The
	 * pivot stays out, the next launch closes it.
	 *
	 * @param path the path command to drive while intaking
	 * @return the command
	 */
	public Command intakeAlong(Command path) {
		return Commands.deadline(
				path,
				Commands.parallel(
						pivot.setPosition(PivotConstants.States.Open),
						roller.setVoltage(RollerConstants.IntakingVoltage)))
				.andThen(roller.setVoltage(RollerConstants.OffVoltage));
	}
}
