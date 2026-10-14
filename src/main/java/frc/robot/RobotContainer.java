// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.overture.lib.gamepads.OverXboxController;
import com.overture.lib.robots.OverContainer;
import com.overture.lib.subsystems.vision.AprilTags;
import com.overture.lib.subsystems.vision.LimelightHelpers;
import com.overture.lib.utils.UtilityFunctions;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.BooleanSupplier;
import frc.robot.Subsystems.Chassis.BLinePaths;
import frc.robot.Subsystems.Chassis.Chassis;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Hood.HoodConstants;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Intake.Pivot.Pivot;
import frc.robot.Subsystems.Intake.Pivot.PivotConstants;
import frc.robot.Subsystems.Intake.Rollers.Roller;
import frc.robot.Subsystems.Intake.Rollers.RollerConstants;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.EmergencyCompress;
import frc.robot.commands.LaunchCommand;
import frc.robot.commands.LaunchConstants;
import frc.robot.commands.LaunchModes;
import frc.robot.commands.autos.AutoCommands;
import frc.robot.commands.autos.AutoRoutines;

public class RobotContainer implements OverContainer {

	// Subsystems
	public final Chassis chassis = new Chassis();
	protected Hood hood = new Hood();
	protected Pivot pivot = new Pivot();
	protected Roller roller = new Roller();
	protected Indexer indexer = new Indexer();
	protected Shooter shooter = new Shooter();

	// Controllers. There is no operator controller, the driver does everything
	private final OverXboxController driver = new OverXboxController(0, 0.20, 0.2);
	private final OverXboxController operator = new OverXboxController(0, 0.20, 0.2);

	// Vision. One field layout shared by the four cameras, each one feeding the
	// chassis pose estimator on its own
	private final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

	private static final String camRightName = "camRight";
	private static final String camLeftName = "camLeft";
	private static final String limelightRightName = "limelight-right";
	private static final String limelightLeftName = "limelight-left";

	// The PhotonVision cameras are not on the robot yet. Uncomment these two and the
	// two photonSeesTag terms in isAnyTagVisible when they are mounted
	// private final AprilTags camRight = new AprilTags(tagLayout, chassis, camRightConfig());
	// private final AprilTags camLeft = new AprilTags(tagLayout, chassis, camLeftConfig());
	private final AprilTags limelightRight = new AprilTags(tagLayout, chassis, limelightRightConfig());
	private final AprilTags limelightLeft = new AprilTags(tagLayout, chassis, limelightLeftConfig());

	// Autonomous
	private final BLinePaths paths = new BLinePaths(chassis);
	private final SendableChooser<Command> autoChooser = new SendableChooser<>();

	// Shooter trims typed on the dashboard, 1.0 means the table as it is
	private static final String hubShooterMultiKey = "LaunchCommand/HubShooterMulti";
	private static final String passShooterMultiKey = "LaunchCommand/PassShooterMulti";

	// Whether the driver is holding one of the intake buttons
	private final BooleanSupplier intakeInUse = driver.leftTrigger().or(driver.x());

	// The building blocks the autos share
	private final AutoCommands autos = new AutoCommands(shooter, hood, chassis, indexer, pivot, roller,
			() -> getShooterMulti(LaunchModes.HUB));

	public RobotContainer() {
		// Only sets them if the dashboard does not already have them, so a trim
		// survives a code restart
		SmartDashboard.setDefaultNumber(hubShooterMultiKey, 1.0);
		SmartDashboard.setDefaultNumber(passShooterMultiKey, 1.0);

		configDriverBindings();
		configOperatorBindings();
		configCharacterizationBindings();
		configureAutos();
	}

	@Override
	public void configDriverBindings() {
		chassis.setDefaultCommand(new DriveCommand(chassis, driver));

		// shooter.setDefaultCommand(shooter.trackVelocity(
		// 		() -> RotationsPerSecond.of(LaunchConstants.DistanceToShooterForHub.get(getHubDistance()))));

		hood.setDefaultCommand(
				Commands.either(hood.holdPosition(HoodConstants.States.Close), hood.HoodHoming(), hood::isHomed));

		// Between shots the indexer keeps the fuel ready at the shooter
		
		indexer.setDefaultCommand(indexer.preloadShooter());

		driver.back().onTrue(Commands.runOnce(() -> {
			chassis.resetHeading(UtilityFunctions.isRedAlliance() ? 180.0 : 0.0);
		}));

		driver.leftTrigger().whileTrue(Commands.parallel(
				pivot.setPosition(PivotConstants.States.Open),
				roller.setVoltage(RollerConstants.IntakingVoltage)));
		driver.leftTrigger().onFalse(roller.setVoltage(RollerConstants.OffVoltage));

		driver.x().whileTrue(Commands.parallel(
				pivot.setPositionSlow(PivotConstants.States.Closed),
				roller.setVoltage(RollerConstants.CompressingVoltage)));
		driver.x().onFalse(Commands.parallel(
				pivot.setPosition(PivotConstants.States.Open),
				roller.setVoltage(RollerConstants.OffVoltage)));

		driver.start().onTrue(hood.HoodHoming());

		driver.rightTrigger().whileTrue(new LaunchCommand(shooter, hood, chassis, indexer, pivot, roller,
				LaunchModes.HUB, () -> getShooterMulti(LaunchModes.HUB), intakeInUse));
		driver.leftBumper().whileTrue(new LaunchCommand(shooter, hood, chassis, indexer, pivot, roller,
				LaunchModes.PASS, () -> getShooterMulti(LaunchModes.PASS), intakeInUse));


		
	}

	@Override
	public void configOperatorBindings() {
		operator.x().whileTrue(new EmergencyCompress(pivot, roller));
		operator.y().onTrue(hood.HoodHoming());
		operator.povUp().onTrue(hood.AddOffset(1));
		operator.povDown().onTrue(hood.AddOffset(-1));
		operator.povRight().onTrue(shooter.AddOffset(1));
		operator.povLeft().onTrue(shooter.AddOffset(-1));
		operator.a().whileTrue(indexer.setVoltage(Volts.of(0.0)));
		operator.b().whileTrue(shooter.setVoltage(Volts.of(0.0)));
	
	}

	@Override
	public void configCharacterizationBindings() {
		// No tuning controller
	}

	private void configureAutos() {
		autoChooser.setDefaultOption("None", Commands.none());

		// Each entry is a path file in deploy/autos/paths. The first path of an auto
		// resets the pose to its start, later ones follow from wherever the robot is

		// Autos are drawn once on the right side of the blue field. Mirrored runs the same
		// file on the left, red is flipped automatically on top of that
		autoChooser.addOption("Double pass right", AutoRoutines.DoublePass(paths, autos, false));
		autoChooser.addOption("Double pass left", AutoRoutines.DoublePass(paths, autos, true));
		autoChooser.addOption("Alternate double pass right", AutoRoutines.AlternateDoublePass(paths, autos, false));
		autoChooser.addOption("Alternate double pass left", AutoRoutines.AlternateDoublePass(paths, autos, true));

		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	// The robot plays with the intake out, Robot.teleopInit schedules this once
	public Command intakeOut() {
		return pivot.setPosition(PivotConstants.States.Open);
	}

	@Override
	public void updateTelemetry() {
		chassis.shuffleboardPeriodic();
		hood.updateTelemetry();
		pivot.updateTelemetry();
		roller.updateTelemetry();
		indexer.updateTelemetry();
		shooter.updateTelemetry();

		driver.updateTelemetry();

		SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
		SmartDashboard.putBoolean("Vision/AnyTagVisible", isAnyTagVisible());
	}

	// Read fresh every time so a trim typed mid-match takes effect on the next loop
	private double getShooterMulti(LaunchModes launchMode) {
		if (launchMode == LaunchModes.PASS) {
			return SmartDashboard.getNumber(passShooterMultiKey, 1.0);
		}
		return SmartDashboard.getNumber(hubShooterMultiKey, 1.0);
	}

	private double getHubDistance() {
		return chassis.getEstimatedPose().getTranslation().getDistance(LaunchConstants.getHubPose());
	}

	// A tag in front of any of the four cameras counts
	public boolean isAnyTagVisible() {
		return LimelightHelpers.getTV(limelightRightName) || LimelightHelpers.getTV(limelightLeftName);
		// || photonSeesTag(camRightName) || photonSeesTag(camLeftName);
	}

	// PhotonVision publishes whether each of its cameras has a target
	private static boolean photonSeesTag(String cameraName) {
		return NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(cameraName)
				.getEntry("hasTarget").getBoolean(false);
	}

	private static AprilTags.Config camRightConfig() {
		AprilTags.Config config = new AprilTags.Config();
		config.backend = AprilTags.VisionBackend.PhotonVision;
		config.cameraName = camRightName;
		config.cameraToRobotSupplier = () -> new Transform3d(
				Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0),
				new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(0.0),
						Units.degreesToRadians(0.0)));
		return config;
	}

	private static AprilTags.Config camLeftConfig() {
		AprilTags.Config config = new AprilTags.Config();
		config.backend = AprilTags.VisionBackend.PhotonVision;
		config.cameraName = camLeftName;
		config.cameraToRobotSupplier = () -> new Transform3d(
				Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0),
				new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(0.0),
						Units.degreesToRadians(0.0)));
		return config;
	}

	private static AprilTags.Config limelightLeftConfig() {
		AprilTags.Config config = new AprilTags.Config();
		config.backend = AprilTags.VisionBackend.Limelight;
		config.cameraName = limelightLeftName;
		config.limelightMode = AprilTags.LimelightMode.MegaTag2;
		config.multiTagStdDevs = new double[] { 0.5, 0.5, 0.5 };
		config.yawCorrectionThreshold = 10.0;
		config.cameraToRobotSupplier = () -> new Transform3d(
				Units.inchesToMeters(-13.659907), Units.inchesToMeters(1.7318223), Units.inchesToMeters(14.785706),
				new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-15.0),
						Units.degreesToRadians(180.0)));
		return config;
	}

	private static AprilTags.Config limelightRightConfig() {
		AprilTags.Config config = new AprilTags.Config();
		config.backend = AprilTags.VisionBackend.Limelight;
		config.cameraName = limelightRightName;
		config.limelightMode = AprilTags.LimelightMode.MegaTag2;
		config.multiTagStdDevs = new double[] { 0.5, 0.5, 0.5 };
		config.yawCorrectionThreshold = 10.0;
		config.cameraToRobotSupplier = () -> new Transform3d(
				Units.inchesToMeters(-13.463993), Units.inchesToMeters(-1.732177), Units.inchesToMeters(14.999332),
				new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-30.0),
						Units.degreesToRadians(180.0)));
		return config;
	}
}
