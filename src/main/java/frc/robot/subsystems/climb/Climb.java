package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged 
public class Climb extends SubsystemBase {

    private SparkMax motor = new SparkMax(Constants.CLIMB_ID, MotorType.kBrushless);

    private double currentVoltage = 0;
    private final double positiveLimit = 5;
    private final double negitiveLimit = -5;
    // double position = motor.getAbsoluteEncoder().getPosition();

    public void motorRun(double voltage) {
        currentVoltage = voltage;
        motor.setVoltage(MathUtil.clamp(voltage, negitiveLimit, positiveLimit));
    }

    public double getCurrentVoltage() {
        return currentVoltage;
    }

    public double getPosition() {
        return motor.getAbsoluteEncoder().getPosition();
    }
}
