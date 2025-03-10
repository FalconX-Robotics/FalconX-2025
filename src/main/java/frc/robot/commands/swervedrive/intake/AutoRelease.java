package frc.robot.commands.swervedrive.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Settings;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swervedrive.Vision;

public class AutoRelease extends Command {
    private Intake intake;
    private Vision vision;
    private Settings settings;

    public AutoRelease(Intake intake, Vision vision, Settings settings) {
        this.intake = intake;
        this.vision = vision;
        this.settings = settings;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (vision.linedUpReef()) {
            intake.set(settings.armSettings.releaseSpeed);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        intake.set(0);
    }
}
