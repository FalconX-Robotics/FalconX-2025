package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Settings;
import frc.robot.subsystems.Intake;

public class Release extends Command {
    Intake intake;
    Settings settings;

    double outSpeed;

    public Release(Intake intake, Settings settings){
        this.intake = intake;
        this.settings = settings;
        this.outSpeed = settings.operatorSettings.releaseSpeed;
        addRequirements(intake);
        setName("Release");
    }

    public Release(Intake intake, double speed) {
        this.intake = intake;
        this.outSpeed = speed;
        addRequirements(intake);
        setName("Release");
    }

    @Override
    public void execute() {
        intake.set(outSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.set(0);
    }

    @Override
    public boolean isFinished() {
        return outSpeed < 0;
    }
}
