package frc.robot.util;

import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Robot;

public class Util {private static LocalDateTime startTime;
    
    public static String getLogFilename() {
        DateTimeFormatter timeFormatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss").withZone(ZoneId.of("UTC"));
        if (Robot.isSimulation()) {
            return ("sim_" + timeFormatter.format(startTime) + ".wpilog");
        }
        return ("robot_" + timeFormatter.format(startTime) + ".wpilog");
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
    public static double remap(double in, double lowIn, double hiIn, double lowOut, double hiOut) {
        return lowOut + (in - lowIn) * (hiOut - lowIn) / (hiIn - lowIn);
    }

    public static Pose3d transformToPose(Transform3d transform) {
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }
}