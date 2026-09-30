package frc.robot.Subsystems.Indexer;

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

public class IndexerConstants {
   
    public class States {
        
        public static Voltage VoltageOFF = Volts.of(0.0);
        public static Voltage VoltageIdle = Volts.of(0.0);
        public static Voltage VoltageON = Volts.of(4);
        
    }

    public static int leaderCanId = 15;
    public static int MotorCanId2 = 14;
    public static int MotorCanId3 = 13;
    public static int MotorCanId4 = 12;

    

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
                    .withNeutralMode(NeutralModeValue.Coast)
                    
            );
    }
}
