package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Settings;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
    double speed;
    Intake intake;

    public RunIntake(Intake intake, double speed) {
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
        setName("GrabCoral");
    }
    
    @Override
    public void initialize() {
        intake.intaking = true;
    }

    @Override
    public void execute() {
        intake.set(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPrimaryMotor(0);
        intake.setFeeder(0);
        intake.intaking = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
