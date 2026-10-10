package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.IndexerConstants;
import frc.robot.Subsystems.Intake.Pivot.Pivot;
import frc.robot.Subsystems.Intake.Pivot.PivotConstants;
import frc.robot.Subsystems.Intake.Rollers.Roller;
import frc.robot.Subsystems.Intake.Rollers.RollerConstants;

/**
 * Feeds the shooter, but only while the shot is ready. While it feeds it also
 * closes the intake, which squeezes the fuel in the hopper towards the shooter.
 * Whenever the shot stops being ready the indexer goes back to preloading and
 * the intake opens again.
 */
public class EjectCommand extends Command {
	private final Indexer indexer;
	private final Pivot pivot;
	private final Roller roller;

	private final BooleanSupplier readyToEject;
	private final BooleanSupplier intakeInUse;

	private boolean compressing = false;

	/**
	 * Builds the eject command.
	 *
	 * @param indexer      the indexer
	 * @param pivot        the intake pivot
	 * @param roller       the intake rollers
	 * @param readyToEject whether the shot is ready, VisionAlignCmd knows
	 * @param intakeInUse  whether the driver is holding one of the intake buttons.
	 *                     While true this command leaves the intake alone
	 */
	public EjectCommand(Indexer indexer, Pivot pivot, Roller roller, BooleanSupplier readyToEject,
			BooleanSupplier intakeInUse) {
		this.indexer = indexer;
		this.pivot = pivot;
		this.roller = roller;
		this.readyToEject = readyToEject;
		this.intakeInUse = intakeInUse;

		// The intake gets moved without being a requirement. That is what lets the
		// driver keep intaking while shooting
		addRequirements(indexer);
	}

	@Override
	public void initialize() {
		compressing = false;
	}

	@Override
	public void execute() {
		if (readyToEject.getAsBoolean()) {
			indexer.setTarget(IndexerConstants.shootVoltage);

			if (!intakeInUse.getAsBoolean()) {
				pivot.setMotor(PivotConstants.States.Closed);
				roller.setTarget(RollerConstants.CompressingVoltage);
				compressing = true;
			}
		} else {
			indexer.setMotorPreload();
			openIntake();
		}
	}

	// Only undoes what this command did, and never takes the intake from the driver
	private void openIntake() {
		if (compressing && !intakeInUse.getAsBoolean()) {
			pivot.setMotor(PivotConstants.States.Open);
			roller.setTarget(RollerConstants.OffVoltage);
		}
		compressing = false;
	}

	@Override
	public void end(boolean interrupted) {
		indexer.setTarget(IndexerConstants.OffVoltage);
		openIntake();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
