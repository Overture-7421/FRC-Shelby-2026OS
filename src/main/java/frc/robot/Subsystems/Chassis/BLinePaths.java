// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

import java.util.HashMap;
import java.util.Map;

/**
 * Hands the chassis to BLine and builds the commands that follow its paths.
 *
 * <p>
 * This is robot code. OvertureLib does not know BLine exists, so this class is
 * the only place the two meet. Build one of these in RobotContainer and ask it
 * for every path command.
 */
public final class BLinePaths {
	private final Chassis chassis;
	private final FollowPath.Builder builder;

	// BLine hands out its target pose, the path points and a few numbers through
	// static hooks. They go to the BLine table so AdvantageScope can draw the target
	// next to the robot pose from Swerve/Chassis/Pose
	private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("BLine");
	private static final Map<String, StructPublisher<Pose2d>> posePublishers = new HashMap<>();
	private static final Map<String, StructArrayPublisher<Translation2d>> pointsPublishers = new HashMap<>();

	static {
		FollowPath.setPoseLoggingConsumer(entry -> posePublishers
				.computeIfAbsent(entry.getFirst(), key -> table.getStructTopic(key, Pose2d.struct).publish())
				.set(entry.getSecond()));
		FollowPath.setTranslationListLoggingConsumer(entry -> pointsPublishers
				.computeIfAbsent(entry.getFirst(), key -> table.getStructArrayTopic(key, Translation2d.struct).publish())
				.set(entry.getSecond()));
		FollowPath.setDoubleLoggingConsumer(entry -> table.getEntry(entry.getFirst()).setDouble(entry.getSecond()));
		FollowPath.setBooleanLoggingConsumer(entry -> table.getEntry(entry.getFirst()).setBoolean(entry.getSecond()));
	}

	/**
	 * Wires BLine to the chassis.
	 *
	 * @param chassis the drivetrain BLine will drive
	 */
	public BLinePaths(Chassis chassis) {
		this.chassis = chassis;

		// The gains are BLine's own starting values, not ones tuned on Shelby.
		builder = new FollowPath.Builder(
				chassis,
				chassis::getEstimatedPose,
				// Robot relative, both the speeds read and the speeds commanded.
				chassis::getCurrentSpeeds,
				chassis::setTargetSpeeds,
				// Translation: how hard to drive at what is left of the path. The robot only
				// starts braking when vmax / kP is left, so kP has to stay under 2 x accel /
				// vmax (4 m/s^2 and 4.5 m/s give 1.8) or it overshoots the end and comes back.
				// 2.2 on the robot: 1.8 crawled the last stretch. The D holds it back only while
				// it closes fast, so it stops at the end instead of coasting past it. Oct 11
				new PIDController(3.5, 0.0, 0.15),
				// Rotation: how hard to turn towards the heading the path asks for.
				new PIDController(6.0, 0.0, 0.3),
				// Cross track: how hard to pull back onto the line between two points. 2 let the

				// robot swing wide through a 90 degree corner, 4 on the robot Oct 11
				new PIDController(4.0, 0.0, 0.0))
				// Paths are drawn on the blue side and flipped when we are red.
				.withDefaultShouldFlip();
	}

	/**
	 * Follows a path from wherever the robot believes it is.
	 *
	 * @param pathName the path file in deploy/autos/paths, without the .json
	 * @return the command
	 */
	public Command follow(String pathName) {
		// The builder remembers its pose reset between builds, so it is set on
		// every build rather than left to whichever method happened to run last.
		return builder.withPoseReset(pose -> {
		}).build(new Path(pathName));
	}

	/**
	 * Places the robot at the start of a path, then follows it. For the first
	 * path of an auto.
	 *
	 * <p>
	 * Only the first. BLine resets the pose every time a command built with a
	 * reset starts, so using this for a later path throws away whatever odometry
	 * and vision worked out on the way there.
	 *
	 * @param pathName the path file in deploy/autos/paths, without the .json
	 * @return the command
	 */
	public Command resetAndFollow(String pathName) {
		// resetPose rather than resetOdometry: it places the simulated robot
		// there as well.
		return builder.withPoseReset(chassis::resetPose).build(new Path(pathName));
	}
}
