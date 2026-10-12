package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.overture.lib.math.InterpolatingTable;
import com.overture.lib.utils.AllianceFlip;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Subsystems.Hood.HoodConstants;

/**
 * Where the robot launches to and with what. Every number here is 2910's, taken
 * from their shooter and passing tables and from the tolerances of their
 * superstructure. The tables get re-measured on our robot.
 */
public final class LaunchConstants {
	private LaunchConstants() {
	}

	// Blue alliance coordinates, flip them for red
	public static final Translation2d HubPose = new Translation2d(4.626, 4.035);
	public static final Translation2d RightPass = new Translation2d(4.0, 2.5);
	public static final Translation2d LeftPass = AllianceFlip.mirrorLeftRight(RightPass);

	// The pass target changes side when the robot crosses the middle of the field
	// by this much, so it does not flicker while driving along the midline
	public static final double FieldMidline = AllianceFlip.getFieldWidth() / 2.0;
	public static final double PassOverlap = 0.2;

	// Distance to the hub in meters -> shooter velocity in rps. Flat until the shooter is tuned,
	// 2910's table is on main
	public static final InterpolatingTable DistanceToShooterForHub = new InterpolatingTable(
			1.3, 23.0,
			5.0, 23.0);

	// Distance to the hub in meters -> hood angle in degrees. Flat until the shooter is tuned,
	// the hood stays down
	public static final InterpolatingTable DistanceToHoodForHub = new InterpolatingTable(
			1.3, 0.0,
			5.0, 0.0);

	// Distance to the pass target in meters -> shooter velocity in rps. Flat until the
	// shooter is tuned, the halved 2910 table is on main
	public static final InterpolatingTable DistanceToShooterForPass = new InterpolatingTable(
			2.0, 23.0,
			10.0, 23.0);

	// Passes leave with the hood all the way up. Down until the shooter is tuned
	public static final Angle HoodForPass = HoodConstants.States.Min;

	// Distance to the pass target in meters -> seconds the fuel spends in the air.
	// Only passes lead the target, the hub is shot at standing still
	public static final InterpolatingTable DistanceToTimeOfFlightForPass = new InterpolatingTable(
			0.5, 0.5,
			6.0, 0.83,
			10.0, 1.1);

	// What has to be true before the indexer is allowed to feed the shooter
	public static final Angle HubChassisTolerance = Degrees.of(4.0);
	public static final LinearVelocity HubSpeedTolerance = MetersPerSecond.of(0.15);
	public static final Distance HubMinimumDistance = Meters.of(1.5);

	public static final Angle PassChassisTolerance = Degrees.of(15.0);
	public static final LinearVelocity PassSpeedTolerance = MetersPerSecond.of(10.0);

	// How much of the driver's translation gets through while the chassis aligns
	public static final double HubTranslationScalar = 1.0;
	public static final double PassTranslationScalar = 0.3;

	// The shooter is at the back of the robot, the intake is the front (+x). Aiming
	// means pointing the back at the target, so the heading target gets this added
	public static final Rotation2d ShooterSide = Rotation2d.fromDegrees(180.0);

	// The heading controller that turns the robot at the target. Tuned on the robot
	public static final double HeadingKP = 6.0;
	public static final double HeadingKD = 0.6;
	public static final double HeadingMaxVelocity = 9.0; // rad/s
	public static final double HeadingMaxAcceleration = 12.0; // rad/s^2

	public static ProfiledPIDController newHeadingController() {
		return new ProfiledPIDController(HeadingKP, 0.0, HeadingKD,
				new TrapezoidProfile.Constraints(HeadingMaxVelocity, HeadingMaxAcceleration));
	}

	public static Translation2d getHubPose() {
		return AllianceFlip.flipIfRed(HubPose);
	}
}
