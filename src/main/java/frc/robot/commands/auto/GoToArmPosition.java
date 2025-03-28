package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class GoToArmPosition extends Command {

    private Arm arm;
    private Elevator elevator;
    
    private Position position;

    public static enum Position {
        INTAKE(0,-2.8256309032440186),
        L2(0,-2.3802669048309326),
        L3(-48.59504699707031,-2.049252986907959),
        LO_ALGAE(0,-2.5969302654266357),
        HI_ALGAE(-44.28524398803711,-2.5969302654266357),
        TRAVEL(0,1.5015838146209717);

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
            //    &&
            //    true;
    }
}
