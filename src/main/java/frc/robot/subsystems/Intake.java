package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.*;

@Logged
public class Intake extends SubsystemBase{
    SparkMax intakeMotor = new SparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
    SparkMax feederMotor = new SparkMax(Constants.FEEDER_MOTOR, MotorType.kBrushless);
    // DigitalInput coralSensor = new DigitalInput(Constants.CORAL_SENSOR);

    DoubleLogEntry intakeVelocityLog = Util.createDoubleLog("intake/Intake Velocity");
    DoubleLogEntry feederVelocityLog = Util.createDoubleLog("intake/Feeder Velocity");
    BooleanLogEntry coralLog = Util.createBooleanLog("intake/coral");
    BooleanLogEntry intakingLog = Util.createBooleanLog("intake/intaking");

    public boolean intaking;
    
    //positive is intake
    public void setPrimaryMotor(double voltage) {
        intakeMotor.set(voltage);
    }

    public double getPrimaryMotor() {
        return intakeMotor.get();
    }

    // positive is intake
    public void setFeeder(double voltage) {
        feederMotor.set(voltage);
    }

    public double getFeeder() {
        return feederMotor.get();
    }
    @Override
    public void periodic() {
        intakingLog.append(intaking);
        intakeVelocityLog.append(intakeMotor.getEncoder().getVelocity());
        feederVelocityLog.append(feederMotor.getEncoder().getVelocity());
    }

    public void set(double speed) {
        setPrimaryMotor(speed);
        setFeeder(speed/4);
    }
}
