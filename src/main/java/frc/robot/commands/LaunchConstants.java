package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.overture.lib.math.InterpolatingTable;
import com.overture.lib.utils.AllianceFlip;

import edu.wpi.first.math.geometry.Translation2d;
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

	// Distance to the hub in meters -> shooter velocity in rps
	public static final InterpolatingTable DistanceToShooterForHub = new InterpolatingTable(
			1.3, 2100.0 / 60.0,
			2.0, 2250.0 / 60.0,
			3.0, 2400.0 / 60.0,
			4.0, 2600.0 / 60.0,
			5.0, 2900.0 / 60.0);

	// Distance to the hub in meters -> hood angle in degrees
	public static final InterpolatingTable DistanceToHoodForHub = new InterpolatingTable(
			1.3, 0.0,
			2.0, 8.0,
			3.0, 17.0,
			4.0, 23.0,
			5.0, 28.0);

	// Distance to the pass target in meters -> shooter velocity in rps
	public static final InterpolatingTable DistanceToShooterForPass = new InterpolatingTable(
			2.0, 2000.0 / 60.0,
			6.0, 3500.0 / 60.0,
			10.0, 6000.0 / 60.0);

	// Passes always leave with the hood all the way up
	public static final Angle HoodForPass = HoodConstants.States.Max;

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

	public static Translation2d getHubPose() {
		return AllianceFlip.flipIfRed(HubPose);
	}
}
