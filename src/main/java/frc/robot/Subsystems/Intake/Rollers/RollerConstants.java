package frc.robot.Subsystems.Intake.Rollers;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.VoltageUnit;
import static edu.wpi.first.units.Units.*;

public class RollerConstants {
  
    protected class Control{

        protected static double kI = 0.0;
            protected static double kD = 0.0;
            protected static double kP = 0.6;
            protected static double kV = 0.12;

            protected static AngularAcceleration AccelerationLimit = RadiansPerSecondPerSecond.of(0.0);
            protected static AngularVelocity CruiseVelocity = RadiansPerSecond.of(0.0);
            protected static Velocity<AngularAccelerationUnit> JerkLimit = RadiansPerSecondPerSecond.per(Second).of(0);
            
            protected static double AcceptedError = 1;

    }

    public class States {
        public static Voltage VoltageOFF = Volts.of(0.0);
        public static Voltage VoltageON = Volts.of(4);
        public static AngularVelocity VelocityOFF = RotationsPerSecond.of(0);
        public static AngularVelocity VelocityON = RotationsPerSecond.of(45);

    }

    public static int motorCanId = 20;

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
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(Control.CruiseVelocity)
                    .withMotionMagicAcceleration(Control.AccelerationLimit)
                    .withMotionMagicJerk(Control.JerkLimit)
            );
    }



}
