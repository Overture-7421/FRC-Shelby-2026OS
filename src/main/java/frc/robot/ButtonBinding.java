package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;


public class ButtonBinding {
    public static final XboxController driver = new XboxController(0);

    public ButtonBinding(){}

    public static void configureBindings(RobotContainer robotContainer) {
        
        Trigger buttonStart = new JoystickButton(driver, XboxController.Button.kStart.value);

        buttonStart
            .whileTrue(robotContainer.hood.HoodHoming())
            .onFalse(robotContainer.hood.setVoltageCommand(Volts.of(0.0)));

    }

}