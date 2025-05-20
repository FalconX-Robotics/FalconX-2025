package frc.robot.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class WaitEntry extends AutoEntry {

    private double time;

    public WaitEntry(String name, ArrayList<Object> params) throws InvalidAutoException {
        super(name, params);
        if (params.size() != 1) {
            throw new InvalidAutoException("Invalid parameter count");
        }
        if (!(params.get(0) instanceof Double)) {
            System.out.println(params.get(0).getClass().getName());
            throw new InvalidAutoException("Invalid parameter type: WaitEntry parameter 1");
        }
        time = (Double) params.get(0);
    }

    @Override
    public Command toCommand() {
        return new WaitCommand(time);
    }
}
