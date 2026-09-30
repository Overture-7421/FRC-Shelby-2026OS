package frc.robot.Subsystems.Hood;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.*;

public class HoodConstants {
    protected class Control {

        protected static double kI = 0.0;
        protected static double kD = 0.0;
        protected static double kP = 0.0;
        protected static double kV = 0.0;

        protected static AngularAcceleration AccelerationLimit = RotationsPerSecondPerSecond.of(0.0);
        protected static AngularVelocity CruiseVelocity = RotationsPerSecond.of(0.0);
        protected static Velocity<AngularAccelerationUnit> JerkLimit = RotationsPerSecondPerSecond.per(Second).of(0);
        
        protected static Angle AcceptedError = Degrees.of(1);            
        protected static Time TimeOut = Seconds.of(1); 
        protected static Angle OffSet = Degrees.of(0); 
        protected static double SensorToMechanismRatio = 1.0;
        protected static double RotorToSensorRatio = 30/35;

    }

    public class States {
            public static Angle Opened = Degree.of(90);
            public static Angle Closed = Degree.of(0);

        }

    public static int motorCanId = 15;
    public static int CCCanId = 14;
    
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
                new VoltageConfigs().withPeakForwardVoltage(12).withPeakReverseVoltage(-12))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withSlot0( 
                new Slot0Configs().withKP(Control.kP).withKV(Control.kV))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withFeedbackRemoteSensorID(CCCanId)
                    .withSensorToMechanismRatio(Control.SensorToMechanismRatio)
                    .withRotorToSensorRatio(Control.RotorToSensorRatio))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(Control.CruiseVelocity)
                    .withMotionMagicAcceleration(Control.AccelerationLimit)
                    .withMotionMagicJerk(Control.JerkLimit)
            );
    }

    public static CANcoderConfiguration CCConfig(){
        CANcoderConfiguration CCConfig = new CANcoderConfiguration();
        CCConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        CCConfig.MagnetSensor.withMagnetOffset(Control.OffSet);
        return CCConfig;

    }
}
