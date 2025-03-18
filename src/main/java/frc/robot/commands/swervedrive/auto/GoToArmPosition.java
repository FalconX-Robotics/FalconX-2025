package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class GoToArmPosition extends Command {

    private Arm arm;
    private Elevator elevator;
    
    private ArmElevatorPosition position;

    public static enum ArmElevatorPosition {
        INTAKE(0,0.7144467287606601),
        L1(0,0.12689228997496202),
        L2(-0.03318296592268683,0.30161103997496275),
        L3(0,0.12805198518303562),
        L4(-0.2593490261170599,-0.05843855526844039),
        HIGH_ALGAE(-0.04734627579649297,0.2903718682613724),
        LOW_ALGAE(-0.10179523018499223,0.09065311826137237),
        TRAVEL(0,1.6544071637908695);

        public final double height;
        public final double angle;

        ArmElevatorPosition(double elevator, double arm) {
            height = elevator;
            angle = arm;
        }
    }

    public GoToArmPosition(ArmElevatorPosition position, Arm arm, Elevator elevator) {
        this.position = position;
        this.arm = arm;
        this.elevator = elevator;
        addRequirements(arm, elevator);
    }

    @Override
    public void initialize() {
        arm.setSetpoint(position.angle);
        // elevator.setSetpoint(position.height);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isNear(position.angle, arm.getAngle(), 0.01) &&
            //    MathUtil.isNear(position.height, elevator.getHeight(), 0.01) &&
               true;
    }
}
