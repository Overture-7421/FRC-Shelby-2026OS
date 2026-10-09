// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.overture.lib.gamepads.OverXboxController;
import com.overture.lib.math.Utils;
import com.overture.lib.utils.UtilityFunctions;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Chassis.Chassis;

/** Port of the C++ DriveCommand: the default teleop drive. */
public class DriveCommand extends Command {
	private final Chassis chassis;
	private final OverXboxController gamepad;

	// Heading control is disabled for now
	// private final ProfiledPIDController headingController = new
	// ProfiledPIDController(
	// 4.75, 0.0, 0.15, new TrapezoidProfile.Constraints(13.0, 18.0 * 2));

	// Stick-driven heading control
	// private final HeadingSpeedsHelper headingSpeedsHelper;

	private final SlewRateLimiter xInput = new SlewRateLimiter(11.0);
	private final SlewRateLimiter yInput = new SlewRateLimiter(11.0);

	private int allianceMulti = 1;
	private final double slowMulti = 1.0;

	/**
	 * While the right bumper is held, translation is capped so the shooter can
	 * solve for it.
	 */
	private static final double kShootWhileMoveMaxSpeed = 2.0;

	/**
	 * Builds the drive command.
	 *
	 * @param chassis   the drivetrain
	 * @param gamepad   the driver's controller
	 */
	public DriveCommand(Chassis chassis, OverXboxController gamepad) {
		this.chassis = chassis;
		this.gamepad = gamepad;
		// Stick-driven heading control
		// this.headingSpeedsHelper = new HeadingSpeedsHelper(headingController,
		// chassis);

		addRequirements(chassis);
	}

	@Override
	public void initialize() {
		allianceMulti = UtilityFunctions.isRedAlliance() ? -1 : 1;
	}

	@Override
	public void execute() {
		// Stick-driven heading control
		//
		// Rotation2d targetAngle = gamepad.getRightStickDirection();
		// if (allianceMulti == -1) {
		// targetAngle = targetAngle.rotateBy(Rotation2d.fromDegrees(180));
		// }
		// double squares = Math.hypot(gamepad.getRightY(), gamepad.getRightX());
		// if (squares > 0.71) {
		// if (!speedHelperMoved) {
		// speedHelperMoved = true;
		// chassis.enableSpeedHelper(headingSpeedsHelper);
		// }
		// } else if (speedHelperMoved) {
		// speedHelperMoved = false;
		// chassis.disableSpeedHelper();
		// }
		// headingSpeedsHelper.setTargetAngle(targetAngle);

		double xSpeed = Utils.applyAxisFilter(allianceMulti * -gamepad.getHID().getRawAxis(1), 0.11, 0.5)
				* chassis.getMaxModuleSpeed()
				* slowMulti;
		double ySpeed = Utils.applyAxisFilter(allianceMulti * -gamepad.getHID().getRawAxis(0), 0.11, 0.5)
				* chassis.getMaxModuleSpeed()
				* slowMulti;

		if (gamepad.getHID().getRightBumperButton()) {
			double vMag = Math.hypot(xSpeed, ySpeed);

			if (vMag > kShootWhileMoveMaxSpeed) {
				// Carried over exactly as the C++ had it: a ratio of squares, not of
				// magnitudes. Capping a
				// vector at vMax normally scales by vMax/vMag; this scales by its square, so it
				// slows the
				// robot more than the cap asks for. It is what the robot drove at Worlds, so it
				// is
				// preserved rather than corrected here.
				double vMaxFactor = (kShootWhileMoveMaxSpeed * kShootWhileMoveMaxSpeed)
						/ (xSpeed * xSpeed + ySpeed * ySpeed);
				xSpeed *= vMaxFactor;
				ySpeed *= vMaxFactor;
			}
		}

		// The C++ wrote this as turns per second and handed it straight to
		// ChassisSpeeds, which takes
		// radians per second; the units library converted on the way. Java carries
		// plain doubles, so
		// the conversion has to be explicit or the robot would spin 2*pi times too
		// slowly.
		double rotationSpeedRadPerSec = Units
				.rotationsToRadians(Utils.applyAxisFilter(gamepad.getRightX(), 0.06, 0.75) * -1.0);

		SmartDashboard.putNumber(
				"DriveCommand/RotationSpeed", Units.radiansToRotations(rotationSpeedRadPerSec));

		ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				xInput.calculate(xSpeed),
				yInput.calculate(ySpeed),
				rotationSpeedRadPerSec,
				chassis.getEstimatedPose().getRotation());

		chassis.setTargetSpeeds(speeds);
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
