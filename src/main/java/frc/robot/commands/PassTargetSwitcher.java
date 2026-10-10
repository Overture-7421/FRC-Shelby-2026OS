package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Port of the C++ PassTargetSwitcher: picks the pass target on the side of the
 * field the robot is on, and only changes its mind once the robot is clearly
 * across the middle.
 */
public class PassTargetSwitcher {
	private enum TargetSide {
		None,
		Left,
		Right
	}

	private final Translation2d leftPassTarget;
	private final Translation2d rightPassTarget;
	private final double overlap;
	private final double midpoint = LaunchConstants.FieldMidline;

	private TargetSide currentTargetSide = TargetSide.None;

	/**
	 * Builds the switcher.
	 *
	 * @param leftPassTarget  the left target, in blue alliance coordinates
	 * @param rightPassTarget the right target, in blue alliance coordinates
	 * @param overlap         how far past the middle the robot has to be before the
	 *                        target changes, in meters
	 */
	public PassTargetSwitcher(Translation2d leftPassTarget, Translation2d rightPassTarget, double overlap) {
		this.leftPassTarget = leftPassTarget;
		this.rightPassTarget = rightPassTarget;
		this.overlap = overlap;
	}

	/**
	 * Returns the target to pass to. It is always in blue alliance coordinates, the
	 * caller flips it when we are red.
	 *
	 * @param chassisPose the robot pose
	 * @param redAlliance whether we are the red alliance
	 * @return the pass target
	 */
	public Translation2d getPassTarget(Pose2d chassisPose, boolean redAlliance) {
		switch (currentTargetSide) {
			case None:
				if (chassisPose.getY() > midpoint) {
					currentTargetSide = TargetSide.Left;
				} else {
					currentTargetSide = TargetSide.Right;
				}
				break;
			case Left:
				if (chassisPose.getY() < midpoint - overlap) {
					currentTargetSide = TargetSide.Right;
				}
				break;
			case Right:
				if (chassisPose.getY() > midpoint + overlap) {
					currentTargetSide = TargetSide.Left;
				}
				break;
			default:
				break;
		}

		if (redAlliance) {
			return currentTargetSide == TargetSide.Left ? rightPassTarget : leftPassTarget;
		} else {
			return currentTargetSide == TargetSide.Left ? leftPassTarget : rightPassTarget;
		}
	}
}
