package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.RunIntake;

public class SetIntakeEntry extends AutoEntry{
    public SetIntakeEntry(String name, Object... params) throws Exception {
        super(name, params);
        if(params.length != 1) {
            throw new Exception("Invalid intake param count");
        }
        if(!(params[0] instanceof Double)) {
            throw new Exception("Invalid parameter type");
        }
        this.speed = (Double) params[0];
    }

    private double maxSpeed = 1;
    private double minSpeed = -1;

    private double speed; 

    public boolean safetyCheck() {
        return (speed <= maxSpeed && speed >= minSpeed);
    }

    public Command toCommand() {
        return new RunIntake(RobotContainer.INSTANCE.intake, speed);
    }
}
