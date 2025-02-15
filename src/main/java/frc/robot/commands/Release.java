package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class Release extends Command {
    Intake intake;
    double velocity;

    public Release(Intake intake, double velocity){
        this.intake = intake;
        this.velocity = velocity;
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
        return velocity > 0;
    }
}
