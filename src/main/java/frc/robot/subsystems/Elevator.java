package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import org.dyn4j.dynamics.Settings;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Elevator
 */
public class Elevator extends SubsystemBase {
    PIDController pid = new PIDController(0, 0, 0); 
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);
    double kG = 0;

    public double motionSetpoint = 0.0;

    private SparkMax elevatorSparkMax = new SparkMax(Constants.ELEVATOR_MOTOR, MotorType.kBrushless);
    private DigitalInput topLimitSwitch = new DigitalInput(Constants.ELEVATOR_TOP_LIMIT_SWITCH);
    private DigitalInput bottomLimitSwitch = new DigitalInput(Constants.ELEVATOR_BOTTOM_LIMIT_SWITCH);

    @Override
    public void periodic() {
        double currentVelocity = elevatorSparkMax.get();
        if (currentVelocity > 0 && topLimitSwitch.get()) setVelocity(0);
        if (currentVelocity < 0 && bottomLimitSwitch.get()) setVelocity(0);
    }

    public void setSetpoint(double position) {
        double currentVelocity = elevatorSparkMax.get();
        motionSetpoint = position;
        pid.setSetpoint(currentVelocity);
        elevatorSparkMax.set(pid.calculate(currentVelocity) + feedforward.calculate(currentVelocity) + kG);
    }

    public void setVelocity(double velocity) {
        elevatorSparkMax.set(velocity);
    }
}