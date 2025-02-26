package frc.robot.commands.swervedrive.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
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
    public void execute() {
        intake.setMotor(settings.armSettings.intakeSpeed);
        intake.setFeeder(settings.armSettings.intakeSpeed/4);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotor(0);
        intake.setFeeder(0);
    }

    @Override
    public boolean isFinished() {
        return intake.hasCoral() && settings.armSettings.intakeSpeed < 0;
    }
}
