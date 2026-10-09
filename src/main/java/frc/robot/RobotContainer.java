// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.Subsystems.Chassis.BLinePaths;
import frc.robot.Subsystems.Chassis.Chassis;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Intake.Pivot.Pivot;
import frc.robot.Subsystems.Intake.Pivot.PivotConstants;
import frc.robot.Subsystems.Intake.Rollers.Roller;
import frc.robot.Subsystems.Intake.Rollers.RollerConstants;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.commands.DriveCommand;

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

	// Vision. One field layout shared by the four cameras, each one feeding the
	// chassis pose estimator on its own
	private final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

	private static final String camRightName = "camRight";
	private static final String camLeftName = "camLeft";
	private static final String limelightUpName = "limelight-up";
	private static final String limelightDownName = "limelight-down";

	private final AprilTags camRight = new AprilTags(tagLayout, chassis, camRightConfig());
	private final AprilTags camLeft = new AprilTags(tagLayout, chassis, camLeftConfig());
	private final AprilTags limelightUp = new AprilTags(tagLayout, chassis, limelightUpConfig());
	private final AprilTags limelightDown = new AprilTags(tagLayout, chassis, limelightDownConfig());

	// Autonomous
	private final BLinePaths paths = new BLinePaths(chassis);
	private final SendableChooser<Command> autoChooser = new SendableChooser<>();

	public RobotContainer() {
		configDriverBindings();
		configOperatorBindings();
		configCharacterizationBindings();
		configureAutos();
	}

	@Override
	public void configDriverBindings() {
		chassis.setDefaultCommand(new DriveCommand(chassis, driver));

		driver.back().onTrue(Commands.runOnce(() -> {
			chassis.resetHeading(UtilityFunctions.isRedAlliance() ? 180.0 : 0.0);
		}));

		driver.leftTrigger().whileTrue(Commands.parallel(
				pivot.setPosition(PivotConstants.States.Open),
				roller.setVoltage(RollerConstants.IntakingVoltage)));
		driver.leftTrigger().onFalse(roller.setVoltage(RollerConstants.OffVoltage));

		driver.x().whileTrue(Commands.parallel(
				pivot.setPosition(PivotConstants.States.Closed),
				roller.setVoltage(RollerConstants.CompressingVoltage)));
		driver.x().onFalse(Commands.parallel(
				pivot.setPosition(PivotConstants.States.Open),
				roller.setVoltage(RollerConstants.OffVoltage)));

		driver.start().onTrue(hood.HoodHoming());
	}

	@Override
	public void configOperatorBindings() {
		// No operator controller
	}

	@Override
	public void configCharacterizationBindings() {
		// No characterization controller
	}

	private void configureAutos() {
		autoChooser.setDefaultOption("None", Commands.none());

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

	// A tag in front of any of the four cameras counts
	public boolean isAnyTagVisible() {
		return LimelightHelpers.getTV(limelightUpName) || LimelightHelpers.getTV(limelightDownName)
				|| photonSeesTag(camRightName) || photonSeesTag(camLeftName);
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

	private static AprilTags.Config limelightUpConfig() {
		AprilTags.Config config = new AprilTags.Config();
		config.backend = AprilTags.VisionBackend.Limelight;
		config.cameraName = limelightUpName;
		config.limelightMode = AprilTags.LimelightMode.MegaTag2;
		config.multiTagStdDevs = new double[] { 0.5, 0.5, 0.5 };
		config.yawCorrectionThreshold = 10.0;
		config.cameraToRobotSupplier = () -> new Transform3d(
				Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0),
				new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(0.0),
						Units.degreesToRadians(0.0)));
		return config;
	}

	private static AprilTags.Config limelightDownConfig() {
		AprilTags.Config config = new AprilTags.Config();
		config.backend = AprilTags.VisionBackend.Limelight;
		config.cameraName = limelightDownName;
		config.limelightMode = AprilTags.LimelightMode.MegaTag2;
		config.multiTagStdDevs = new double[] { 0.5, 0.5, 0.5 };
		config.yawCorrectionThreshold = 10.0;
		config.cameraToRobotSupplier = () -> new Transform3d(
				Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0),
				new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(0.0),
						Units.degreesToRadians(0.0)));
		return config;
	}
}
