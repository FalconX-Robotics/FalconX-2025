package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class GoToArmPosition extends Command {

    private Arm arm;
    private Elevator elevator;
    
    private Position position;

    public static enum Position {
        INTAKE(0,0),
        L1(0,0),
        L2(0,0),
        L3(0,0),
        L4(0,0),
        HIGH_ALGAE(0,0),
        LOW_ALGAE(0,0),
        TRAVEL(0,0);

        public final double height;
        public final double angle;

        Position(double elevator, double arm) {
            height = elevator;
            angle = arm;
        }
    }

    public GoToArmPosition(Position position, Arm arm, Elevator elevator) {
        this.position = position;
        this.arm = arm;
        this.elevator = elevator;
        addRequirements(arm, elevator);
    }

    @Override
    public void initialize() {
        arm.setSetpoint(position.angle);
        elevator.setSetpoint(position.height);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isNear(position.angle, arm.getAngle(), 0.01) &&
               MathUtil.isNear(position.height, elevator.getHeight(), 0.01);
    }
}
