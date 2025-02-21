package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    private SparkMax motor = new SparkMax(Constants.CLIMB_ID, MotorType.kBrushless);

    private double positiveLimit = 5;
    private double negitiveLimit = -5;

    double position = motor.getAbsoluteEncoder().getPosition();

    public void motorRun(double voltage){
        double finalVoltage = voltage;

        boolean beyondNegative = position <= negitiveLimit;
        boolean beyonePositive = position >= positiveLimit;

        if (beyondNegative && (voltage < 0)){
            finalVoltage = 0;
        }
        
        if (beyonePositive && (voltage > 0)){
            finalVoltage = 0;
        }
        
        motor.setVoltage(finalVoltage);
    }
}
