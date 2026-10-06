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

	public static Voltage offVoltage = Volts.of(0.0);
	public static Voltage shootVoltage = Volts.of(4);
	public static Voltage preloadVoltage = Volts.of(2);

	public static int leaderCanId = 15;
	public static int MotorCanId2 = 14;
	public static int MotorCanId3 = 13;
	public static int MotorCanId4 = 12;

	public static int shooterCanRangeId = 11;
	public static int hopperCanRangeId = 10;

	public static final double GearRatio = (1 / 1);

	public static TalonFXConfiguration motorConfig() {
		return new TalonFXConfiguration()
				.withCurrentLimits(
						new CurrentLimitsConfigs()
								.withStatorCurrentLimitEnable(true)
								.withStatorCurrentLimit(120)
								.withSupplyCurrentLimitEnable(true)
								.withStatorCurrentLimit(40)
								.withSupplyCurrentLowerLimit(40)
								.withSupplyCurrentLowerTime(0.05))
				.withVoltage(
						new VoltageConfigs()
								.withPeakForwardVoltage(12)
								.withPeakReverseVoltage(-12))
				.withMotorOutput(
						new MotorOutputConfigs()
								.withInverted(InvertedValue.CounterClockwise_Positive)
								.withNeutralMode(NeutralModeValue.Coast))
				.withFeedback(
						new FeedbackConfigs()
								.withSensorToMechanismRatio(GearRatio));
	}

}
