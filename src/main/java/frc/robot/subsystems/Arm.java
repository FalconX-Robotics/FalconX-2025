package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;

public class Arm extends SubsystemBase {
    SparkMax armMotor = new SparkMax(25, MotorType.kBrushless);
    Settings m_settings;

    public Arm (Settings settings) {
        m_settings = settings;
    }

   

    
    @Override
    public void periodic() {
        //idk yet bro
    }
}
