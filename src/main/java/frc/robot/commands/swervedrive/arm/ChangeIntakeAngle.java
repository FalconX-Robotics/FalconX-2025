package frc.robot.commands.swervedrive.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ChangeIntakeAngle extends Command {
    Arm arm;
    CommandXboxController xbox;

    public ChangeIntakeAngle(Arm arm, CommandXboxController xbox) {
        this.arm = arm;
        this.xbox = xbox;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setVelocity(xbox.getRightY());

      // Get the mass, angle, & radius of the claw.
      final double clawMass = Constants.CLAW_MASS;
      final double clawAngle = this.arm.getAngle();
      final double clawRadius = Constants.CLAW_RADIUS;

      // Get the X value of the left joystick.
      final double joystickValue = xbox.getRightY();

      // Define the acceleration due to gravity.
      final double accelerationDueToGravity = -9.8;

      // Calculate the torque.
      final double gravityTorque = accelerationDueToGravity * clawMass * clawRadius * Math.cos( Units.degreesToRadians( clawAngle ) );
      final double magicNumber = 1;

      // Declare the velocity to use for the arm motor.
      final double armMotorVelocity = gravityTorque * magicNumber;
      
      // Get the arm velocity based on the calculated factors.
      double calculatedArmVelocity = armMotorVelocity + ( joystickValue * Constants.CLAW_SPEED );

      // Declare the positive and negative limits.
      final double positiveLimit = 1;
      final double negativeLimit = -1;

      // Apply the limits to the arm.
      final boolean goingBeyondPositive = ( (clawAngle / 360) > positiveLimit ) && ( joystickValue > 0 );
      final boolean goingBeyondNegative = ( (clawAngle / 360) < negativeLimit ) && ( joystickValue < 0 );
      if ( goingBeyondPositive || goingBeyondNegative ) calculatedArmVelocity = 0;

      this.arm.setVelocity( calculatedArmVelocity );
    }

    @Override
    public void end(boolean interrupted) {
        arm.setVelocity(0);
    }
}

