package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.reduxrobotics.sensors.canandgyro.Canandgyro.Settings;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    CANSparkMax armMotor = new CANSparkMax(25, MotorType.kBrushless);
    RelativeEncoder m_armEncoder;
    Settings m_settings;

    public Arm (Settings settings) {
        m_settings = settings;

        m_armEncoder = armMotor.getEncoder();
        /**TODO: get the thing to put into the conversion */
        m_armEncoder.setPositionConversionFactor(-1.0);
    }

   

    
    @Override
    public void periodic() {
        //idk yet bro
    }
}
