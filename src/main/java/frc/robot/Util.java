package frc.robot;

import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.ProtobufLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
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

    public static DoubleLogEntry createDoubleLog(String name) {
        return new DoubleLogEntry(DataLogManager.getLog(), name);
    }
    public static IntegerLogEntry createIntLog(String name) {
        return new IntegerLogEntry(DataLogManager.getLog(), name);
    }
    public static BooleanLogEntry createBooleanLog(String name) {
        return new BooleanLogEntry(DataLogManager.getLog(), name);
    }

    public static double poundsToKilos(double pounds) {
        return pounds * 0.453592;
    }
}