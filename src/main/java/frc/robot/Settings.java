package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;

public class Settings {
    private CommandXboxController driveController, operatorController;
    public OperatorSettings armSettings;

    public Settings (CommandXboxController driveController, CommandXboxController operatorController) {
        this.operatorController = operatorController;
        this.driveController = driveController;

        armSettings = new OperatorSettings();
    }
    /**Controller bindings and such for controlling arm and arm adjacent parts (eg:intake and elevator) */
    public class OperatorSettings {
        public Trigger coralIntakeButton       = operatorController.a();
        public Trigger armAngleButton          = operatorController.b();
        public Trigger realeaseButton    = operatorController.x();
        public Trigger overrideArm = new Trigger(() -> {return Math.abs(operatorController.getRightY()) > 0.1;});

        public Trigger climbButton = operatorController.leftBumper();
        public Trigger unClimbButton = operatorController.rightBumper();

        public double intakeSpeed = 0.5;
        public double releaseSpeed = -0.5;

        public double climbSpeed = 5;
    }
}
