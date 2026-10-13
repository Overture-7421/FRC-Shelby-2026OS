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
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;

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

	private static final double MinVelocityMetersPerSec = 0.3;
	private static final double MinVelocityDegPerSec = 20.0;
	private final FollowPath.Builder builder;

	private final Map<String, Path> loadedPaths = new HashMap<>();

	// The first follow after a boot paid for class loading and JIT at auto start, a
	// good part of a second before the robot moved. So the first path built runs
	// through a throwaway follower at boot, outputs discarded, while the robot is
	// still disabled. Same idea as PathPlanner's warm-up command
	private boolean warmedUp = false;

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
				chassis::getCurrentSpeeds,
				chassis::setTargetSpeeds,
				
				new PIDController(3.5, 0.0, 0.15),
				new PIDController(6.0, 0.0, 0.3),

				new PIDController(4.0, 0.0, 0.0))
				// Paths are drawn on the blue side and flipped when we are red.
				.withDefaultShouldFlip();
	}

	/**
	 * Follows a path from wherever the robot believes it is, as drawn.
	 *
	 * @param pathName the path file in deploy/autos/paths, without the .json
	 * @return the command
	 */
	public Command follow(String pathName) {
		return follow(pathName, false);
	}

	/**
	 * Follows a path from wherever the robot believes it is.
	 *
	 * @param pathName the path file in deploy/autos/paths, without the .json
	 * @param mirrored true reflects the path across the length of the field, so a
	 *                 path drawn on the right side runs on the left. Headings
	 *                 mirror with it
	 * @return the command
	 */
	public Command follow(String pathName, boolean mirrored) {
		return build(pathName, mirrored, false);
	}

	/**
	 * Places the robot at the start of a path, then follows it, as drawn. For the
	 * first path of an auto.
	 *
	 * @param pathName the path file in deploy/autos/paths, without the .json
	 * @return the command
	 */
	public Command resetAndFollow(String pathName) {
		return resetAndFollow(pathName, false);
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
	 * @param mirrored true reflects the path across the length of the field
	 * @return the command
	 */
	public Command resetAndFollow(String pathName, boolean mirrored) {
		return build(pathName, mirrored, true);
	}

	
	private Command build(String pathName, boolean mirrored, boolean resetPose) {
		// resetPose rather than resetOdometry: it places the simulated robot there as well
		Consumer<Pose2d> reset = resetPose ? chassis::resetPose : pose -> {
		};

		Path path = load(pathName);
		if (!warmedUp) {
			warmUp(path);
			warmedUp = true;
		}
		return Commands.defer(() -> builder
				.withPoseReset(reset)
				.withShouldMirror(() -> mirrored)
				.build(path), Set.of(chassis));
	}

	private void warmUp(Path path) {
		FollowPath follower = new FollowPath.Builder(
				chassis,
				chassis::getEstimatedPose,
				chassis::getCurrentSpeeds,
				speeds -> {
				},
				new PIDController(1.0, 0.0, 0.0),
				new PIDController(1.0, 0.0, 0.0),
				new PIDController(1.0, 0.0, 0.0))
				.withPoseReset(pose -> {
				})
				.build(path);
		follower.initialize();
		for (int i = 0; i < 50; i++) {
			follower.execute();
		}
		follower.end(true);
	}

	private Path load(String pathName) {
		return loadedPaths.computeIfAbsent(pathName, BLinePaths::parse);
	}

	private static Path parse(String pathName) {
		Path path = new Path(pathName);
		Path.PathConstraints constraints = path.getPathConstraints();
		if (constraints.getMinVelocityMetersPerSec().isEmpty()) {
			constraints.setMinVelocityMetersPerSec(MinVelocityMetersPerSec);
		}
		if (constraints.getMinVelocityDegPerSec().isEmpty()) {
			constraints.setMinVelocityDegPerSec(MinVelocityDegPerSec);
		}
		path.setPathConstraints(constraints);
		return path;
	}
}
