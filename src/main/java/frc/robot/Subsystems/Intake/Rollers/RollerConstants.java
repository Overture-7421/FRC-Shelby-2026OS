package frc.robot.Subsystems.Intake.Rollers;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;

public class RollerConstants {

	public static Voltage OffVoltage = Volts.of(0.0);
	public static Voltage IntakingVoltage = Volts.of(12);
	public static Voltage OutTakingVoltage = Volts.of(-4);
	public static Voltage CompressingVoltage = Volts.of(1.8);

	public static int motorCanId = 32; // Right, the leader
	public static int MotorCanId2 = 31; // Left, follows the leader inverted

	public static final double GearRatio = (1 / 1);

	public static TalonFXConfiguration motorConfig() {
		return new TalonFXConfiguration()
				.withCurrentLimits(
						new CurrentLimitsConfigs()
								.withStatorCurrentLimitEnable(true)
								.withStatorCurrentLimit(100)
								.withSupplyCurrentLimitEnable(true)
								.withSupplyCurrentLimit(40))
				.withVoltage(
						new VoltageConfigs()
								.withPeakForwardVoltage(12)
								.withPeakReverseVoltage(-12))
				.withMotorOutput(
						new MotorOutputConfigs()
								.withInverted(InvertedValue.Clockwise_Positive)
								.withNeutralMode(NeutralModeValue.Brake))
				.withFeedback(
						new FeedbackConfigs()
								.withSensorToMechanismRatio(GearRatio));
	}

	
}
