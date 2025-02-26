package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util;

public class Intake extends SubsystemBase{
    SparkMax intakeMotor = new SparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
    SparkMax feederMotor = new SparkMax(Constants.FEEDER_MOTOR, MotorType.kBrushless);
    DigitalInput coralSensor = new DigitalInput(Constants.CORAL_SENSOR);

    DoubleLogEntry intakeVelocityLog = Util.createDoubleLog("intake/Intake Velocity");
    DoubleLogEntry feederVelocityLog = Util.createDoubleLog("intake/Feeder Velocity");
    BooleanLogEntry coralLog = Util.createBooleanLog("intake/coral");
    
    //positive is intake
    public void setMotor(double voltage) {
        intakeMotor.set(voltage);
    }

    // positive is intake
    public void setFeeder(double voltage) {
        feederMotor.set(voltage);
    }

    public boolean hasCoral() {
        return coralSensor.get();
    }
    @Override
    public void periodic() {
        intakeVelocityLog.append(intakeMotor.getEncoder().getVelocity());
        feederVelocityLog.append(feederMotor.getEncoder().getVelocity());
        coralLog.append(hasCoral());
    }
}
