package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;

public class Settings {
    private XboxController m_driveController, m_armController;
    public ArmSettings armSettings;

    public Settings (XboxController driveController, XboxController armController) {
        m_armController = armController;
        m_driveController = driveController;

        armSettings = new ArmSettings();
    }
    /**Controller bindings and such for controlling arm and arm adjacent parts (eg:intake and elevator) */
    public class ArmSettings {
        public Trigger intakeButton     = new JoystickButton(m_armController, Button.kA.value);
        public Trigger lowerArmButton   = new JoystickButton(m_armController, Button.kB.value);
        public Trigger upperArmButton   = new JoystickButton(m_armController, Button.kX.value);
        
    }
}
