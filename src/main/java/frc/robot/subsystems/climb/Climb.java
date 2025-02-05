package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase{

    private SparkMax motor = new SparkMax(Constants.CLIMB_ID, MotorType.kBrushless);

    private double positiveLimit = 5;
    private double negitiveLimit = -5;

    public void motorRun(double voltage){
        motor.setVoltage(voltage);
        if (negitiveLimit>=motor.getAbsoluteEncoder().getPosition() && voltage<0){
        
            motor.setVoltage(0);
        }
        if (positiveLimit<=motor.getAbsoluteEncoder().getPosition() && voltage>0){

            motor.setVoltage(0);
        }
    }
    
}
