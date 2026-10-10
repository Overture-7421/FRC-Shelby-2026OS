package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.overture.lib.math.ChassisAccels;
import com.overture.lib.math.TargetingWhileMoving;
import com.overture.lib.subsystems.swerve.HeadingSpeedsHelper;
import com.overture.lib.subsystems.swerve.SpeedsHelper;
import com.overture.lib.utils.AllianceFlip;
import com.overture.lib.utils.UtilityFunctions;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Chassis.Chassis;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Shooter.Shooter;

/**
 * Points the chassis at the target and gets the shooter and the hood ready for
 * it, for as long as it runs. It is our VisionAlignCmd doing what 2910's
 * superstructure does while scoring or passing: same tables, same conditions to
 * call the shot ready.
 *
 * <p>
 * It does not feed the shooter. EjectCommand does, asking isReadyToLaunch().
 */
public class VisionAlignCmd extends Command {
	private final Shooter shooter;
	private final Hood hood;
	private final Chassis chassis;
	private final LaunchModes launchMode;

	private final DoubleSupplier multiSupplier;
	private final BooleanSupplier tagVisible;

	// Same heading controller the C++ VisionAlignCmd aimed with
	private final ProfiledPIDController headingController = new ProfiledPIDController(
			6.0, 0.0, 0.0, new TrapezoidProfile.Constraints(9.0, 4.0));
	private final HeadingSpeedsHelper headingSpeedsHelper;

	// The heading helper only takes over the rotation. This one also slows the
	// translation the driver asks for, which 2910 does while passing
	private final SpeedsHelper alignSpeedsHelper;
	private double translationScalar = 1.0;

	private final TargetingWhileMoving targetWhileMoving = new TargetingWhileMoving(
			LaunchConstants.DistanceToTimeOfFlightForPass, 0.01);

	private final PassTargetSwitcher passTargetSwitcher = new PassTargetSwitcher(
			LaunchConstants.LeftPass, LaunchConstants.RightPass, LaunchConstants.PassOverlap);

	private double chassisError = 0.0;
	private boolean tagSeen = false;
	private boolean readyToLaunch = false;

	private final StructPublisher<Translation2d> targetPublisher = NetworkTableInstance.getDefault()
			.getStructTopic("SmartDashboard/MovingTarget", Translation2d.struct).publish();

	/**
	 * Builds the align command.
	 *
	 * @param shooter       the shooter
	 * @param hood          the hood
	 * @param chassis       the drivetrain
	 * @param launchMode    whether this is a hub shot or a pass. Each launch
	 *                      button builds its own
	 * @param multiSupplier multiplier for the shooter velocity
	 * @param tagVisible    whether a camera is looking at an AprilTag. A hub shot
	 *                      waits to see one
	 */
	public VisionAlignCmd(Shooter shooter, Hood hood, Chassis chassis, LaunchModes launchMode,
			DoubleSupplier multiSupplier, BooleanSupplier tagVisible) {
		this.shooter = shooter;
		this.hood = hood;
		this.chassis = chassis;
		this.launchMode = launchMode;
		this.multiSupplier = multiSupplier;
		this.tagVisible = tagVisible;

		headingSpeedsHelper = new HeadingSpeedsHelper(headingController, chassis);
		alignSpeedsHelper = new SpeedsHelper() {
			@Override
			public void alterSpeed(ChassisSpeeds inputSpeed) {
				inputSpeed.vxMetersPerSecond *= translationScalar;
				inputSpeed.vyMetersPerSecond *= translationScalar;
				headingSpeedsHelper.alterSpeed(inputSpeed);
			}

			@Override
			public void initialize() {
				headingSpeedsHelper.initialize();
			}
		};

		// The chassis is not a requirement, DriveCommand keeps moving the robot
		addRequirements(shooter, hood);
	}

	@Override
	public void initialize() {
		tagSeen = false;
		readyToLaunch = false;

		chassis.enableSpeedHelper(alignSpeedsHelper);
	}

	@Override
	public void execute() {
		Pose2d chassisPose = chassis.getEstimatedPose();
		boolean redAlliance = UtilityFunctions.isRedAlliance();
		Translation2d targetCoords;

		if (launchMode == LaunchModes.PASS) {
			targetCoords = passTargetSwitcher.getPassTarget(chassisPose, redAlliance);
		} else {
			targetCoords = LaunchConstants.HubPose;
		}
		if (redAlliance) {
			targetCoords = AllianceFlip.flip(targetCoords);
		}

		// Passes are thrown on the move, so they aim at where the target will seem
		// to be once the fuel lands. The hub is shot at standing still
		Translation2d movingGoalLocation = targetCoords;
		if (launchMode == LaunchModes.PASS) {
			targetWhileMoving.setTargetLocation(targetCoords);

			ChassisSpeeds speed = ChassisSpeeds.fromRobotRelativeSpeeds(chassis.getCurrentSpeeds(),
					chassisPose.getRotation());
			ChassisAccels accel = ChassisAccels.fromRobotRelativeAccels(chassis.getCurrentAccels(),
					chassisPose.getRotation());
			movingGoalLocation = targetWhileMoving.getMovingTarget(chassisPose, speed, accel);
		}

		// The shooter looks to the front of the robot
		Rotation2d targetAngle = movingGoalLocation.minus(chassisPose.getTranslation()).getAngle();
		headingSpeedsHelper.setTargetAngle(targetAngle);

		double distanceToTarget = chassisPose.getTranslation().getDistance(movingGoalLocation);
		SmartDashboard.putNumber("LaunchCommand/DistanceTarget", distanceToTarget);

		double hoodAngle;
		double shooterSpeed;

		if (launchMode == LaunchModes.PASS) {
			hoodAngle = LaunchConstants.HoodForPass.in(Degrees);
			shooterSpeed = LaunchConstants.DistanceToShooterForPass.get(distanceToTarget);
			translationScalar = LaunchConstants.PassTranslationScalar;
		} else {
			hoodAngle = LaunchConstants.DistanceToHoodForHub.get(distanceToTarget);
			shooterSpeed = LaunchConstants.DistanceToShooterForHub.get(distanceToTarget);
			translationScalar = LaunchConstants.HubTranslationScalar;
		}

		double multi = multiSupplier.getAsDouble();
		hood.setMotor(Degrees.of(hoodAngle));
		shooter.setMotor(RotationsPerSecond.of(shooterSpeed * multi));
		SmartDashboard.putString("LaunchCommand/LaunchMode", launchMode.toString());
		SmartDashboard.putNumber("LaunchCommand/ShooterMulti", multi);

		chassisError = Math.abs(targetAngle.minus(chassisPose.getRotation()).getDegrees());

		ChassisSpeeds currentSpeeds = chassis.getCurrentSpeeds();
		double chassisSpeed = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

		boolean shotConditions = shooter.isAtTarget() && hood.isAtTarget();
		boolean check;

		if (launchMode == LaunchModes.PASS) {
			check = shotConditions
					&& chassisError < LaunchConstants.PassChassisTolerance.in(Degrees)
					&& chassisSpeed <= LaunchConstants.PassSpeedTolerance.in(MetersPerSecond);
		} else {
			check = shotConditions
					&& distanceToTarget >= LaunchConstants.HubMinimumDistance.in(Meters)
					&& chassisError < LaunchConstants.HubChassisTolerance.in(Degrees)
					&& chassisSpeed <= LaunchConstants.HubSpeedTolerance.in(MetersPerSecond);
		}

		// A hub shot also wants a camera to have seen a tag once everything else is
		// in place, so the pose it aims with is fresh. One sighting is enough, it
		// holds until the shot conditions are lost
		if (check && launchMode == LaunchModes.HUB) {
			tagSeen = tagSeen || tagVisible.getAsBoolean();
		} else {
			tagSeen = false;
		}

		readyToLaunch = check && (launchMode == LaunchModes.PASS || tagSeen);

		SmartDashboard.putNumber("LaunchCommand/ChassisError", chassisError);
		SmartDashboard.putBoolean("LaunchCommand/ShotConditions", shotConditions);
		SmartDashboard.putBoolean("LaunchCommand/TagSeen", tagSeen);
		SmartDashboard.putBoolean("LaunchCommand/ReadyToLaunch", readyToLaunch);
		targetPublisher.set(movingGoalLocation);
	}

	/**
	 * Whether everything is in place for the indexer to feed the shooter.
	 *
	 * @return true while the shot is ready
	 */
	public boolean isReadyToLaunch() {
		return readyToLaunch;
	}

	@Override
	public void end(boolean interrupted) {
		readyToLaunch = false;
		SmartDashboard.putBoolean("LaunchCommand/ReadyToLaunch", readyToLaunch);

		chassis.disableSpeedHelper();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
