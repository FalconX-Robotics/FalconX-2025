package frc.robot.auto;

import java.util.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class AutoEntry {
    protected ArrayList<Object> parameters = new ArrayList<>();
    protected String name;

    public AutoEntry(String name, Object... params) {
        this.name = name;
        for (Object param : params) {
            this.parameters.add(param);
        }
    }

    public boolean safetyCheck() {
        return false;
    }

    public Command toCommand() {
        return Commands.none();
    }
}