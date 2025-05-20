package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Settings;
import frc.robot.subsystems.Intake;

public class GrabCoral extends Command {
    Intake intake;
    Settings settings;

    public GrabCoral(Intake intake, Settings settings) {
        this.settings = settings;
        this.intake = intake;
        addRequirements(intake);
        setName("GrabCoral");
    }

    @Override
    public void initialize() {
        intake.intaking = true;
    }

    @Override
    public void execute() {
        intake.set(settings.operatorSettings.intakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // intake.setPrimaryMotor(0);
        // intake.setFeeder(0);
        // intake.intaking = false;
    }

    @Override
    public boolean isFinished() {
        return true;
        // return settings.operatorSettings.intakeSpeed > 0;
    }
}
