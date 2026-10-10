package frc.robot.Subsystems.Intake.Pivot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import static edu.wpi.first.units.Units.*;

public class PivotConstants {

	protected class Control {

		protected static double kP = 0.0;
		protected static double kV = 0.0;

        protected static AngularAcceleration AccelerationLimit = RadiansPerSecondPerSecond.of(0.0);
        protected static AngularVelocity CruiseVelocity = RadiansPerSecond.of(0.0);
        protected static Velocity<AngularAccelerationUnit> JerkLimit = RadiansPerSecondPerSecond.per(Second).of(0);
        
        protected static Angle AcceptedError = Degrees.of(2); 
		protected static Angle EncoderOffSet = Degrees.of(0);           
        protected static Angle OffSet = Degrees.of(0); 
		
    }

    public class States {

        public static Angle Open = Degree.of(114);
        public static Angle Closed = Degree.of(10);

    }

	public static int motorCanId = 29;
	public static int CCCanId = 30;

	public static final double RotorToSensorRatio = 40.0;

	public static TalonFXConfiguration motorConfig() {
		return new TalonFXConfiguration()
				.withCurrentLimits(
						new CurrentLimitsConfigs()
								.withStatorCurrentLimitEnable(true)
								.withStatorCurrentLimit(75)
								.withSupplyCurrentLimitEnable(true)
								.withSupplyCurrentLimit(30))
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
								.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
								.withFeedbackRemoteSensorID(CCCanId)
								.withRotorToSensorRatio(RotorToSensorRatio));
	}

	public static CANcoderConfiguration CCConfig() {
		CANcoderConfiguration CCConfig = new CANcoderConfiguration();
		CCConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
		CCConfig.MagnetSensor.withMagnetOffset(Control.EncoderOffSet);
		return CCConfig;
	}

}
