package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ChangeIntakeAngle extends Command {
    Arm m_arm;
    double velocity;

    public ChangeIntakeAngle(Arm arm, double velocity) {
        m_arm = arm;
        this.velocity = velocity;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        m_arm.setArmMotor(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.setArmMotor(0);
    }
}

