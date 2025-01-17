package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class Release extends Command {
    Intake m_intake;
    double m_velocity;

    public Release(Intake intake, double velocity){
        m_intake = intake;
        m_velocity = velocity;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        m_intake.setMotor(m_velocity);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setMotor(0);
    }
}
