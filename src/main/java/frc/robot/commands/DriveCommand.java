// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.overture.lib.gamepads.OverXboxController;
import com.overture.lib.math.Utils;
import com.overture.lib.utils.UtilityFunctions;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Chassis.Chassis;

public class DriveCommand extends Command {
	private final Chassis chassis;
	private final OverXboxController gamepad;

	/**
	 * How fast the robot may speed up or change direction, in m/s per second. This
	 * is what pulls current from the battery.
	 */
	private static final double kTranslationAccelLimit = 6.0;

	/**
	 * How fast the robot may slow down, in m/s per second. Braking takes energy out
	 * rather than putting it in, so it can be quicker without costing battery; past
	 * about 11 the tires slide instead.
	 */
	private static final double kTranslationDecelLimit = 11.0;

	/**
	 * How far the limited velocity may drift from how the robot is really moving,
	 * in m/s, before it is snapped back to the measured speed. The limiter only
	 * remembers what it asked for; after hitting something the robot is stopped
	 * but the limiter still believes it is at speed, so a reversal first spent
	 * half a second braking a velocity that no longer existed. Normal driving stays
	 * well inside this, since open loop the wheels run a little under the request.
	 */
	private static final double kReseedThreshold = 1.0;

	private static final double kTranslationDeadband = 0.11;
	private static final double kTranslationExpo = 0.2;

	/**
	 * Rotation always available, even at full translation, in rad/s. Above what
	 * translation leaves free, rotation is capped so the modules do not saturate;
	 * this much is kept anyway and paid for with a little translation. At full
	 * translation the drivers asked for more turn than 0.3 gave; 0.45 costs about a
	 * fifth of the translation while both sticks are pinned.
	 */
	private static final double kMinRotationRadPerSec = Units.rotationsToRadians(0.45);

	/**
	 * The velocity actually requested, after the slew limit. One limit on the whole
	 * vector rather than one per axis: per axis, diagonals accelerated √2 faster
	 * and a
	 * change of direction drove a curve while each axis caught up on its own.
	 */
	private double limitedVx = 0.0;
	private double limitedVy = 0.0;
	private double lastTimestamp = 0.0;

	private int allianceMulti = 1;
	private final double slowMulti = 1.0;

	/**
	 * Builds the drive command.
	 *
	 * @param chassis the drivetrain
	 * @param gamepad the driver's controller
	 */
	public DriveCommand(Chassis chassis, OverXboxController gamepad) {
		this.chassis = chassis;
		this.gamepad = gamepad;

		addRequirements(chassis);
	}

	@Override
	public void initialize() {
		allianceMulti = UtilityFunctions.isRedAlliance() ? -1 : 1;

		// Start the limit from how the robot is actually moving, so taking over from an
		// auto or another command does not jump or brake to a stop
		ChassisSpeeds current = ChassisSpeeds.fromRobotRelativeSpeeds(
				chassis.getCurrentSpeeds(), chassis.getEstimatedPose().getRotation());
		limitedVx = current.vxMetersPerSecond;
		limitedVy = current.vyMetersPerSecond;
		lastTimestamp = MathSharedStore.getTimestamp();
	}

	@Override
	public void execute() {
		// Both sticks shaped as one vector: the deadband and the expo act on how far
		// the
		// stick is pushed, and the direction is kept exactly as the stick points.
		// Shaping
		// each axis on its own made the deadband a square that cut small diagonal
		// moves,
		// and bent the direction, since the curve changes the ratio between the axes.
		double stickX = allianceMulti * -gamepad.getHID().getRawAxis(1);
		double stickY = allianceMulti * -gamepad.getHID().getRawAxis(0);
		// Many sticks read a little past 1 in the corners
		double stickMagnitude = Math.min(1.0, Math.hypot(stickX, stickY));
		double shapedMagnitude = Utils.applyAxisFilter(stickMagnitude, kTranslationDeadband, kTranslationExpo);
		double stickScale = stickMagnitude > 0.0
				? shapedMagnitude / Math.hypot(stickX, stickY) * chassis.getMaxModuleSpeed() * slowMulti
				: 0.0;

		// Shooting while moving needs no cap here: a pass scales this translation
		// itself
		// through VisionAlignCmd's speeds helper
		limitTranslation(stickX * stickScale, stickY * stickScale);

		double rotationSpeedRadPerSec = Units
				.rotationsToRadians(Utils.applyAxisFilter(gamepad.getRightX(), 0.06, 0.3) * -1.0);

		// Translation keeps priority. Spinning adds ω times the drive base radius to
		// every module's speed, and once a module would pass its maximum the chassis
		// scales the whole request down, so spinning at speed used to slow the robot.
		// Rotation only gets the module speed translation leaves free, with a floor
		double translationSpeed = Math.hypot(limitedVx, limitedVy);
		double freeRotation = Math.max(0.0, chassis.getMaxModuleSpeed() - translationSpeed)
				/ chassis.getDriveBaseRadius();
		double rotationLimit = Math.max(freeRotation, kMinRotationRadPerSec);
		rotationSpeedRadPerSec = Math.max(-rotationLimit, Math.min(rotationLimit, rotationSpeedRadPerSec));

		SmartDashboard.putNumber(
				"DriveCommand/RotationSpeed", Units.radiansToRotations(rotationSpeedRadPerSec));

		ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				limitedVx,
				limitedVy,
				rotationSpeedRadPerSec,
				chassis.getEstimatedPose().getRotation());

		chassis.setTargetSpeeds(speeds);
	}

	/**
	 * Moves the requested velocity toward the target. The part of the change that
	 * slows the robot down is limited by the decel limit; speeding up and turning
	 * the direction of travel share the accel limit.
	 */
	private void limitTranslation(double targetVx, double targetVy) {
		double now = MathSharedStore.getTimestamp();
		double dt = now - lastTimestamp;
		lastTimestamp = now;
		double accelBudget = kTranslationAccelLimit * dt;
		double decelBudget = kTranslationDecelLimit * dt;

		// The limiter remembers what it asked for, not what the robot did. When the two
		// part ways, as against a wall, start again from the measured speed
		ChassisSpeeds measured = ChassisSpeeds.fromRobotRelativeSpeeds(
				chassis.getCurrentSpeeds(), chassis.getEstimatedPose().getRotation());
		if (Math.hypot(limitedVx - measured.vxMetersPerSecond,
				limitedVy - measured.vyMetersPerSecond) > kReseedThreshold) {
			limitedVx = measured.vxMetersPerSecond;
			limitedVy = measured.vyMetersPerSecond;
		}

		double changeX = targetVx - limitedVx;
		double changeY = targetVy - limitedVy;
		double speed = Math.hypot(limitedVx, limitedVy);

		if (speed < 1e-6) {
			// Standing still: everything is speeding up
			double change = Math.hypot(changeX, changeY);
			if (change > accelBudget) {
				changeX *= accelBudget / change;
				changeY *= accelBudget / change;
			}
		} else {
			// Split the change along the current velocity (negative slows down, positive
			// speeds up) and across it (turns the direction of travel)
			double unitX = limitedVx / speed;
			double unitY = limitedVy / speed;
			double along = changeX * unitX + changeY * unitY;
			double acrossX = changeX - along * unitX;
			double acrossY = changeY - along * unitY;

			// Braking stops at zero: a reversal brakes first, then speeds up the other way
			// at the accel limit on the next loops
			double slowing = Math.max(-Math.min(decelBudget, speed), Math.min(0.0, along));

			double speedingUp = Math.max(0.0, along);
			double growth = Math.hypot(speedingUp, Math.hypot(acrossX, acrossY));
			double growthScale = growth > accelBudget ? accelBudget / growth : 1.0;

			double alongResult = slowing + speedingUp * growthScale;
			changeX = alongResult * unitX + acrossX * growthScale;
			changeY = alongResult * unitY + acrossY * growthScale;
		}
		limitedVx += changeX;
		limitedVy += changeY;
	}

	@Override
	public void end(boolean interrupted) {
		chassis.disableSpeedHelper();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
