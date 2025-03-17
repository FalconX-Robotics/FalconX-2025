package frc.robot.commands.swervedrive.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Settings;
import frc.robot.subsystems.Intake;
import frc.robot.util.Util;

public class GrabCoral extends Command {
    Intake intake;
    Settings settings;
    CommandXboxController xbox;

    public GrabCoral(Intake intake, Settings settings, CommandXboxController xbox) {
        this.settings = settings;
        this.intake = intake;
        this.xbox = xbox;
        addRequirements(intake);
        setName("GrabCoral");
    }

    @Override
    public void initialize() {
        intake.intaking = true;
    }

    @Override
    public void execute() {
        intake.set(settings.armSettings.intakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPrimaryMotor(0);
        intake.setFeeder(0);
        intake.intaking = false;
    }

    @Override
    public boolean isFinished() {
        return settings.armSettings.intakeSpeed > 0;
    }
}
