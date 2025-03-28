package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Settings;
import frc.robot.subsystems.Intake;

public class Release extends Command {
    Intake intake;
    Settings settings;

    public Release(Intake intake, Settings settings){
        this.intake = intake;
        this.settings = settings;
        addRequirements(intake);
        setName("Release");
    }

    @Override
    public void execute() {
        intake.set(settings.armSettings.releaseSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.set(0);
    }

    @Override
    public boolean isFinished() {
        return settings.armSettings.releaseSpeed < 0;
    }
}
