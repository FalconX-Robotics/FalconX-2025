package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class GrabCoral extends Command {
    Intake m_intake;
    double m_velocity;
    DigitalInput m_intakeLimitSwitch = new DigitalInput(1);

    public GrabCoral(Intake intake, double velocity) {
        m_velocity = velocity;
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (m_intakeLimitSwitch.get()){
            m_intake.setMotor(0);
        } else {
            m_intake.setMotor(m_velocity);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setMotor(0);
    }
}
