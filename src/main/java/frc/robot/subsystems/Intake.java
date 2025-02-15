package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    SparkMax intakeMotor = new SparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
    SparkMax feederMotor = new SparkMax(Constants.FEEDER_MOTOR, MotorType.kBrushless);
    // DigitalInput coralSensor = new DigitalInput(Constants.CORAL_SENSOR);

    public Intake() {
    }
    public void setMotor(double speed) {
        intakeMotor.set(speed);
    }

    // public double getSpeedX() {
    //     return grabberSparkMaxX.get();
    // }
    // public double getSpeedY() {
    //     return grabberSparkMaxY.get();
    // }

    @Override
    public void periodic() {
        
    }
}
