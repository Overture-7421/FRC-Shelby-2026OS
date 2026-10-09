package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.AngularAccelerationUnit;
import static edu.wpi.first.units.Units.*;

public class ShooterConstants {

	protected class Control {

		protected static double kP = 0.0;
		protected static double kV = 0.0;

		protected static AngularAcceleration AccelerationLimit = RotationsPerSecondPerSecond.of(0.0);
		protected static AngularVelocity CruiseVelocity = RotationsPerSecond.of(0.0);
		protected static Velocity<AngularAccelerationUnit> JerkLimit = RotationsPerSecondPerSecond.per(Second).of(0);

		protected static double AcceptedError = 1;
		protected static double SensorToMechanismRatio = 1.6666666666;

	}

	public static Voltage VoltageOFF = Volts.of(0.0);

	public static int leaderCanId = 25;
	public static int MotorCanId2 = 26;
	public static int MotorCanId3 = 27;
	public static int MotorCanId4 = 28;

	public static final double GearRatio = (1 / 1);

	public static TalonFXConfiguration motorConfig() {
		return new TalonFXConfiguration()
				.withCurrentLimits(
						new CurrentLimitsConfigs()
								.withStatorCurrentLimitEnable(true)
								.withStatorCurrentLimit(60)
								.withSupplyCurrentLimitEnable(true)
								.withSupplyCurrentLimit(35))
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
								.withSensorToMechanismRatio(Control.SensorToMechanismRatio))
				.withSlot0(
						new Slot0Configs()
								.withKP(Control.kP)
								.withKV(Control.kV))
				.withMotionMagic(
						new MotionMagicConfigs()
								.withMotionMagicCruiseVelocity(Control.CruiseVelocity)
								.withMotionMagicAcceleration(Control.AccelerationLimit)
								.withMotionMagicJerk(Control.JerkLimit));
	}
}
