package frc.robot;

import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;

import frc.robot.Robot;

public class Util {
    private static LocalDateTime startTime;
    
    public static String getLogFilename() {
        DateTimeFormatter m_TimeFormatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss").withZone(ZoneId.of("UTC"));
        if (Robot.isSimulation()) {
            return ("sim_" + m_TimeFormatter.format(startTime) + ".wpilog");
        }
        return ("robot_" + m_TimeFormatter.format(startTime) + ".wpilog");
    }

    public static void setStartTime(LocalDateTime time) {
        startTime = time;
    }
}