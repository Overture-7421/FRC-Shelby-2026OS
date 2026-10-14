package frc.robot.Subsystems.Indexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;

public class IndexerConstants {

	public static Voltage OffVoltage = Volts.of(0.0);
	public static Voltage shootVoltage = Volts.of(6);
	public static Voltage preloadVoltage = Volts.of(1);

	// Seconds the hopper sensor has to stay covered before the hopper counts as full
	public static final double fuelDebouncingTime = 0.25;

	public static final double hopperFuelDistance = 0.10;
	; // Meters
	public static final double shooterFuelDistance = 0.35; // Meters

	public static final int leaderCanId = 20; // Front left
	public static final int MotorCanId2 = 21; // Front right
	public static final int MotorCanId3 = 22; // Back right
	public static final int MotorCanId4 = 23; // Back left

	// One CANrange looks at the hopper and the other at the shooter feed, right before
	// the wheels
	public static int hopperCanRangeId = 33;
	public static int shooterCanRangeId = 34;

	public static final double GearRatio = (1 / 1);

	public static TalonFXConfiguration motorConfig() {
		return new TalonFXConfiguration()
				.withCurrentLimits(
						new CurrentLimitsConfigs()
								.withStatorCurrentLimitEnable(true)
								.withStatorCurrentLimit(80)
								.withSupplyCurrentLimitEnable(true)
								.withSupplyCurrentLimit(40))
				.withVoltage(
						new VoltageConfigs()
								.withPeakForwardVoltage(12)
								.withPeakReverseVoltage(-12))
				.withMotorOutput(
						new MotorOutputConfigs()
								.withInverted(InvertedValue.CounterClockwise_Positive)
								.withNeutralMode(NeutralModeValue.Brake))
				.withFeedback(
						new FeedbackConfigs()
								.withSensorToMechanismRatio(GearRatio));
	}


}
