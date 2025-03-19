package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged 
public class Climber extends SubsystemBase {

    private SparkMax motor = new SparkMax(Constants.CLIMB_ID, MotorType.kBrushless);

    private double currentVoltage = 0;
    private final double positiveLimit = 5;
    private final double negitiveLimit = -5;
    // double position = motor.getAbsoluteEncoder().getPosition(); // old code; do we need this?

    public void motorRun(double voltage) {
        currentVoltage = voltage;
        // TODO: Should this be clamped? we found some code that suggested it should be
        motor.setVoltage(MathUtil.clamp(voltage, negitiveLimit, positiveLimit));
    }

    public double getCurrentVoltage() {
        return currentVoltage;
    }

    public double getPosition() {
        return motor.getAbsoluteEncoder().getPosition();
    }
}
