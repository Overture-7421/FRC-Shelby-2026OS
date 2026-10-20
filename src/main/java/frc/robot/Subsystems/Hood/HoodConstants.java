package frc.robot.Subsystems.Hood;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;

public class HoodConstants {

	protected class Control {

		// Starting points, 2910 method on our ratio: kP = 5.0 x 94.9 per hood rotation,
		// kV = 12 V over the free speed at the hood (about 1.29 rps). Tune on the robot
		protected static double kP = 474.5;
		protected static double kV = 9.3;

		// 3000 deg/s^2 and 7000 RPM at the motor, in hood rotations
		protected static AngularAcceleration AccelerationLimit = RotationsPerSecondPerSecond.of(8.33);
		protected static AngularVelocity CruiseVelocity = RotationsPerSecond.of(1.23);
		protected static Velocity<AngularAccelerationUnit> JerkLimit = RotationsPerSecondPerSecond.per(Second).of(0);

		protected static Angle AcceptedError = Degrees.of(1.5);


		protected static Voltage HomingVoltage = Volts.of(-4.0);
		protected static AngularVelocity HomingSpeedThreshold = DegreesPerSecond.of(1.0);
		protected static double HomingSettleTime = 0.2; // Seconds
		protected static Angle HomedPosition = Degrees.of(0.0);

	}

	public class States {
		public static Angle Max = Degree.of(37);
		public static Angle Min = Degree.of(1);
		public static Angle Close = Degree.of(1);

	}

	public static int motorCanId = 24;

	public static final double SensorToMechanismRatio = 94.9;

	public static TalonFXConfiguration motorConfig() {
		return new TalonFXConfiguration()
				.withCurrentLimits(
						new CurrentLimitsConfigs()
								.withStatorCurrentLimitEnable(true)
								.withStatorCurrentLimit(60)
								.withSupplyCurrentLimitEnable(true)
								.withSupplyCurrentLimit(20))
				.withVoltage(
						new VoltageConfigs()
								.withPeakForwardVoltage(12)
								.withPeakReverseVoltage(-12))
				.withMotorOutput(
						new MotorOutputConfigs()
								.withInverted(InvertedValue.Clockwise_Positive)
								.withNeutralMode(NeutralModeValue.Brake))
				.withSlot0(
						new Slot0Configs()
								.withKP(Control.kP)
								.withKV(Control.kV))
				.withMotionMagic(
						new MotionMagicConfigs()
								.withMotionMagicCruiseVelocity(Control.CruiseVelocity)
								.withMotionMagicAcceleration(Control.AccelerationLimit)
								.withMotionMagicJerk(Control.JerkLimit))
				.withFeedback(
						new FeedbackConfigs()
								.withSensorToMechanismRatio(SensorToMechanismRatio));
	}
}
