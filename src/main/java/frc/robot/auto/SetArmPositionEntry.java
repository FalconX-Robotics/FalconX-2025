package frc.robot.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.GoToArmPosition;
import frc.robot.commands.auto.GoToArmPosition.Position;

public class SetArmPositionEntry extends AutoEntry{
    public SetArmPositionEntry(String name, ArrayList<Object> params) throws Exception {
        super(name, params);
        if(params.size() != 1) {
            throw new InvalidAutoException("Invalid intake param count");
        }
        if(!(params.get(0) instanceof Double)) {
            throw new InvalidAutoException("Invalid parameter type: SetArmPositionEntry parameter 1");
        }
        int position = ((Double) params.get(0)).intValue();
        if(position == 0) {
            this.target = Position.TRAVEL;
        }
        else if(position == 1) {
            this.target = Position.L2;
        }
        else if(position == 2) {
            this.target = Position.L3;
        }
        else if(position == 3) {
            this.target = Position.INTAKE;
        }
        else {
            System.out.println("Invalid Arm Position");
            throw new InvalidAutoException("Invalid Paramter Value: SetArmPosition paramater 1");
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
