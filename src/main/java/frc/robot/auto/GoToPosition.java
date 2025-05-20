package frc.robot.auto;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class GoToPosition extends AutoEntry {
    Position position = Position.START;
    public GoToPosition(String name, ArrayList<Object> objects) throws InvalidAutoException {
        super(name, objects);
        if (objects.size() != 1) {
            throw new InvalidAutoException("Invalid Go To Position param count");
        }
        if (!(objects.get(0) instanceof String)) {
            throw new InvalidAutoException("Invalid parameter type: GoToPosition parameter 1");
        }
        String positionParam = ((String) objects.get(0)).toLowerCase();
        if (positionParam.equals("left")) {
            this.position = Position.LEFT;
        }
        else if (positionParam.equals("right")) {
            this.position = Position.LEFT;
        }
        else if (positionParam.equals("source")) {
            this.position = Position.SOURCE;
        }
        else if (positionParam.equals("start")) {
            this.position = Position.START;
        }
        else {
            System.out.println("Invalid Go to Position");
            throw new InvalidAutoException("Invalid Paramter Value: GoToPosition paramater 1");
        }
    }

    enum Position {
        LEFT,
        RIGHT,
        SOURCE,
        START
    }

    @Override
    public boolean safetyCheck() {
        return true;
    }

    @Override
    public Command toCommand() {
        PathPlannerPath path;
        final double maxSpeedmPs = 0.5;
        PathConstraints constraints = new PathConstraints(maxSpeedmPs, 1.0,
            Units.degreesToRadians(Constants.MAX_ANGULAR_ACCELERATION), Units.degreesToRadians(Constants.MAX_ANGULAR_VELOCITY));
        try {
            switch (position) {
                case LEFT:
                    path = PathPlannerPath.fromPathFile("MFD To Left");
                    break;
                case RIGHT:
                    path = PathPlannerPath.fromPathFile("MFD To Right");
                    break;
                case SOURCE:
                    path = PathPlannerPath.fromPathFile("MFD To Source");
                    break;
                case START:
                    path = PathPlannerPath.fromPathFile("MFD To Start");
                    break;
                default:
                    path = PathPlannerPath.fromPathFile("MFD To Start");
            }
        } catch (Exception e) {
            e.printStackTrace();
            path = null;
        }
        return AutoBuilder.pathfindThenFollowPath(path, constraints);
    }
}
