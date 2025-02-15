package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class GrabCoral extends Command {
    Intake intake;
    double velocity;

    public GrabCoral(Intake intake, double velocity) {
        this.velocity = velocity;
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setMotor(velocity);
        intake.setFeeder(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotor(0);
        intake.setFeeder(0);
    }

    @Override
    public boolean isFinished() {
        return intake.hasCoral() && velocity < 0;
    }
}
