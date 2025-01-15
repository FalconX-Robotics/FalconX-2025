package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    CANSparkMax armMotor = new CANSparkMax(25, MotorType.kBrushless);
    // CANSparkMax lowerArmMotor = new CANSparkMax(24, MotorType.kBrushless);
    // CANSparkMax upperArmMotor = new CANSparkMax(25, MotorType.kBrushless);

    public Arm () {
        //empty
    }

    public void setArmMotor(double speed) {
        armMotor.set(speed);
    }

    // public void setUpperArmMotor(double speed) {
    //     upperArmMotor.set(speed);
    // }

    // public void setLowerArmMotor(double speed) {
    //     lowerArmMotor.set(speed);
    // }

    @Override
    public void periodic() {
        //idk yet bro
    }
}
