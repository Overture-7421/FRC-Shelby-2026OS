package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.CloseAndCompress;
import frc.robot.Commands.OpenCommand;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Intake.Pivot.Pivot;
import frc.robot.Subsystems.Intake.Pivot.PivotConstants;
import frc.robot.Subsystems.Intake.Rollers.Roller;
import frc.robot.Subsystems.Intake.Rollers.RollerConstants;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import com.overture.lib.robots.RobotConstants;




public class ButtonBinding {
    public static final XboxController driver = new XboxController(0);

    public ButtonBinding(){
        
    }

    public static void configureBindings(Hood hood, Roller roller, Pivot pivot) {
        
        Trigger buttonStart = new JoystickButton(driver, XboxController.Button.kStart.value);
        Trigger buttonLeftTrigger = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
        Trigger buttonX = new JoystickButton(driver, XboxController.Button.kX.value);

        buttonStart
            .whileTrue(hood.HoodHoming())
            .onFalse(hood.setVoltageCommand(Volts.of(0.0)));
        
        buttonLeftTrigger
            .whileTrue(OpenCommand.openCommand(pivot, roller))
            .onFalse(roller.setVoltage(RollerConstants.IntakingVoltage));

        buttonX
            .whileTrue(CloseAndCompress.closeAndCompres(pivot, roller))
            .onFalse(OpenCommand.openCommand(pivot, roller));
        

    }

}