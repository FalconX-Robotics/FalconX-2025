package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged 
public class Climb extends SubsystemBase {

    private SparkMax motor = new SparkMax(Constants.CLIMB_ID, MotorType.kBrushless);

    private double voltage;

    private double positiveLimit = 5;
    private double negitiveLimit = -5;

    double position = motor.getAbsoluteEncoder().getPosition();

    public void motorRun(double voltage) {
        this.voltage = voltage;
        motor.setVoltage(voltage);
    }
}
