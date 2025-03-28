package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.*;

/**
 * Elevator
 */
public class Elevator extends SubsystemBase {
    PIDController pid = new PIDController(0.1, 0, 0.); 
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);
    double kG = 0;

    public double motionSetpoint = 0.0;

    private SparkMax elevatorSparkMax = new SparkMax(Constants.ELEVATOR_MOTOR, MotorType.kBrushless);
    private DigitalInput topLimitSwitch = new DigitalInput(Constants.ELEVATOR_HIGH_SWITCH);
    private DigitalInput bottomLimitSwitch = new DigitalInput(Constants.ELEVATOR_LOW_SWITCH);
    Trigger onBottomSwitch = new Trigger(()-> {return !bottomLimitSwitch.get();});

    private DoubleLogEntry heightLog = Util.createDoubleLog("elevator/height");
    private DoubleLogEntry setpointLog = Util.createDoubleLog("elevator/setpoint");

    public boolean override = false;

    public Elevator() {
        pid.setSetpoint(0);
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        elevatorSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // elevatorSparkMax.getEncoder().setPosition(0);
        // onBottomSwitch.onTrue(Commands.run(()-> {elevatorSparkMax.getEncoder().setPosition(0);}, new Subsystem[0]));
    }

    @Override
    public void periodic() {
        double motorOut = pid.calculate(getHeight());
        

        if (!bottomLimitSwitch.get()) {
             motorOut = Math.min(0, motorOut);
        }
        if (atLimit()) {
            motorOut = Math.max(0.1, motorOut);
        }

        motorOut = MathUtil.clamp(motorOut, -0.8, 0.8);

        SmartDashboard.putBoolean("Bottom Limit", !bottomLimitSwitch.get());
        SmartDashboard.putBoolean("Upper Limit", atLimit());
        SmartDashboard.putNumber("Elevator Speed", elevatorSparkMax.get());
        SmartDashboard.putNumber("Elevator Out", motorOut);

        elevatorSparkMax.set(motorOut);

        SmartDashboard.putNumber("Elevator Setpoint", pid.getSetpoint());
        SmartDashboard.putNumber("Elevator Position", getHeight());
        setpointLog.append(pid.getSetpoint());
        heightLog.append(getHeight());
            
        // if (!bottomLimitSwitch.get()) elevatorSparkMax.set(Math.min(0, elevatorSparkMax.get()));
    }

    public void setSetpoint(double position) {
        pid.setSetpoint(position);
    }
    public void setInput(double input) {
        pid.setSetpoint(pid.getSetpoint() + input * 2.5/4.0);
    }

    //idk what units these are
    public double getHeight() {
        return elevatorSparkMax.getEncoder().getPosition();
    }

    public boolean atLimit() {
        return false;
        // return topLimitSwitch.get();
    }
}