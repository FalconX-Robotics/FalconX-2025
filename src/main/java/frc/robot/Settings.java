package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;

public class Settings {
    private XboxController driveController, operatorController;
    public ArmSettings armSettings;

    public Settings (XboxController driveController, XboxController operatorController) {
        this.operatorController = operatorController;
        this.driveController = driveController;

        armSettings = new ArmSettings();
    }
    /**Controller bindings and such for controlling arm and arm adjacent parts (eg:intake and elevator) */
    public class ArmSettings {
        public Trigger coralIntakeButton       = new JoystickButton(operatorController, Button.kA.value);
        public Trigger armAngleButton          = new JoystickButton(operatorController, Button.kB.value);
        public Trigger realeaseButton    = new JoystickButton(operatorController, Button.kX.value);
        public Trigger overrideArm = new Trigger(() -> {return Math.abs(operatorController.getRightY()) > 0.1;});

        public double intakeSpeed = 0.5;
        public double releaseSpeed = 0.5;
    }
}
