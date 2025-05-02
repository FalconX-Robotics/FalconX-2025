package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.GoToArmPosition;
import frc.robot.commands.auto.GoToArmPosition.Position;

public class SetArmPositionEntry extends AutoEntry{
    public SetArmPositionEntry(String name, Object... params) throws Exception {
        super(name, params);
        if(params.length != 1) {
            throw new Exception("Invalid intake param count");
        }
        if(!(params[0] instanceof String)) {
            throw new Exception("Invalid parameter type");
        }
        if(params[0].equals("Intake")) {
            this.target = Position.INTAKE;
        }
        if(params[0].equals("L2")) {
            this.target = Position.L2;
        }
        if(params[0].equals("L3")) {
            this.target = Position.L3;
        }
        if(params[0].equals("Low Algae")) {
            this.target = Position.LO_ALGAE;
        }
        if(params[0].equals("High Algae")) {
            this.target = Position.HI_ALGAE;
        }
        if(params[0].equals("Down")) {
            this.target = Position.TRAVEL;
        }
    }

    Position target;


    public boolean safetyCheck() {
        return true;
    }

    public Command toCommand() {
        return new GoToArmPosition(target, RobotContainer.INSTANCE.arm, RobotContainer.INSTANCE.elevator);
    }
}
