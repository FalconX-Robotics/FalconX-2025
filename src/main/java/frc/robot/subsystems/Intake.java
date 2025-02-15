package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    SparkMax intakeMotor = new SparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
    SparkMax feederMotor = new SparkMax(Constants.FEEDER_MOTOR, MotorType.kBrushless);
    DigitalInput coralSensor = new DigitalInput(Constants.CORAL_SENSOR);
    
    //positive is intake
    public void setMotor(double voltage) {
        intakeMotor.set(voltage);
    }

    // positive is intake
    public void setFeeder(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    public boolean hasCoral() {
        return coralSensor.get();
    }
}
