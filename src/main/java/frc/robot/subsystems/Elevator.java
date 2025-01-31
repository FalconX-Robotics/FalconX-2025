package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import org.dyn4j.dynamics.Settings;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Elevator
 */
public class Elevator extends SubsystemBase {
    public SparkMax elevatorSparkMax = new SparkMax(30, MotorType.kBrushless);
    public Settings elevatorSettings = new Settings();

    public Elevator() {}

    public void setVelocity(double velocity) {
        elevatorSparkMax.set(velocity);
    }

}