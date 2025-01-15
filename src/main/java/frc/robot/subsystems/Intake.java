package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    CANSparkMax grabberSparkMaxX = new CANSparkMax(22, MotorType.kBrushless);
    CANSparkMax grabberSparkMaxY = new CANSparkMax(21, MotorType.kBrushless);

    public Intake() {
        grabberSparkMaxY.follow(grabberSparkMaxX);
        grabberSparkMaxY.setInverted(true);
        //nothing
    }
    public void setMotor(double speed) {
        grabberSparkMaxX.set(speed);
    }

    // public double getSpeedX() {
    //     return grabberSparkMaxX.get();
    // }
    // public double getSpeedY() {
    //     return grabberSparkMaxY.get();
    // }

    @Override
    public void periodic() {
        //idk yet gimme a min
    }
}
