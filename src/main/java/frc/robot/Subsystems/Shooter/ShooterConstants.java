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

		// Volts per rps of error. 0.35 is what 2910 runs, our wheel is heavier so the
		// same gain recovered from a fuel hit too slowly, Oct 13 logs. Back off if it hunts
		protected static double kP = 1.0;
		protected static double kV = 0.2;

		protected static AngularAcceleration AccelerationLimit = RotationsPerSecondPerSecond.of(400.0);
		protected static AngularVelocity CruiseVelocity = RotationsPerSecond.of(0.0);
		protected static Velocity<AngularAccelerationUnit> JerkLimit = RotationsPerSecondPerSecond.per(Second).of(0);

		protected static AngularVelocity AcceptedError = RotationsPerSecond.of(1.5);

		protected static AngularVelocity FeedingError = RotationsPerSecond.of(10.0);
		protected static double SensorToMechanismRatio = 1.6666666666;

	}

	public static Voltage OffVoltage = Volts.of(0.0);

	// Speed held between launches, all the time. Just under the lowest
	// shot in the tables (hub 22 rps at 1.3 m, pass 19), so a launch only adds the
	// last part of the spin-up and never has to brake the flywheel. Holding costs
	// about 20 A for the four motors; tracking the hub distance while driving cost
	// 45 to 50 A and spinning up from zero 1.1 s per launch, Oct 31 logs
	public static AngularVelocity IdleSpeed = RotationsPerSecond.of(20.0);

	public static int leaderCanId = 25; // Top left
	public static int MotorCanId2 = 26; // Bottom left
	public static int MotorCanId3 = 27; // Top right
	public static int MotorCanId4 = 28; // Bottom right

	public static TalonFXConfiguration motorConfig() {
		return new TalonFXConfiguration()
				.withCurrentLimits(
						new CurrentLimitsConfigs()
								.withStatorCurrentLimitEnable(true)
								// Recovery torque after a fuel hit. Peaks were 41 to 48 A at 60, Oct 14 logs
								.withStatorCurrentLimit(80)
								.withSupplyCurrentLimitEnable(true)
								.withSupplyCurrentLimit(35))
				.withVoltage(
						new VoltageConfigs()
								.withPeakForwardVoltage(12)
								.withPeakReverseVoltage(0)) // The flywheel never spins backwards
				.withMotorOutput(
						new MotorOutputConfigs()
								.withInverted(InvertedValue.Clockwise_Positive)
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
