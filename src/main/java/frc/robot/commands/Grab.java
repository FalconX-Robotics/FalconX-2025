package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class Grab extends Command {
    Intake m_intake;
    double m_velocity;

    public Grab(Intake intake, double velocity) {
        m_velocity = velocity;
        m_intake = intake;
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
