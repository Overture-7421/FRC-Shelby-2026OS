package frc.robot;

import com.ctre.phoenix6.CANBus;

/**
 * Robot-wide numerical and boolean constants.
 */
public final class Constants {
	private Constants() {
	}

	public static final class RobotConstants {
		private RobotConstants() {
		}

		/** The roboRIO's own CAN bus. */
		public static final CANBus rio = new CANBus("rio");

		/** The CANivore carrying the drivetrain. */
		public static final CANBus overCANivore = new CANBus("OverCANivore");
	}
}
