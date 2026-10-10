package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.Chassis.Chassis;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Intake.Pivot.Pivot;
import frc.robot.Subsystems.Intake.Rollers.Roller;
import frc.robot.Subsystems.Shooter.Shooter;

/**
 * Everything the robot does while a launch button is held. VisionAlignCmd aims
 * and prepares the shot, EjectCommand feeds whenever VisionAlignCmd says the
 * shot is ready. Hub or pass is fixed when the command is built: the hub button
 * and the pass button each hold their own.
 */
public class LaunchCommand extends ParallelCommandGroup {

	/**
	 * Builds the launch command.
	 *
	 * @param shooter       the shooter
	 * @param hood          the hood
	 * @param chassis       the drivetrain
	 * @param indexer       the indexer
	 * @param pivot         the intake pivot
	 * @param roller        the intake rollers
	 * @param launchMode    whether this is a hub shot or a pass
	 * @param multiSupplier multiplier for the shooter velocity
	 * @param tagVisible    whether a camera is looking at an AprilTag
	 * @param intakeInUse   whether the driver is holding an intake button
	 */
	public LaunchCommand(Shooter shooter, Hood hood, Chassis chassis, Indexer indexer, Pivot pivot, Roller roller,
			LaunchModes launchMode, DoubleSupplier multiSupplier, BooleanSupplier tagVisible,
			BooleanSupplier intakeInUse) {
		VisionAlignCmd visionAlignCmd = new VisionAlignCmd(shooter, hood, chassis, launchMode,
				multiSupplier, tagVisible);

		// The order matters. The align command has to decide whether the shot is
		// ready before the eject command asks
		addCommands(
				visionAlignCmd,
				new EjectCommand(indexer, pivot, roller, visionAlignCmd::isReadyToLaunch, intakeInUse));
	}
}
