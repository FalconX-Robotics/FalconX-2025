package frc.robot.auto;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.security.KeyStore.Entry;
import java.util.ArrayList;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.GoToArmPosition;
import frc.robot.commands.auto.GoToArmPosition.Position;

public class AutoParser {

    public static ArrayList<AutoEntry> entries = new ArrayList<>();

    public static Command parseAuto(String filePath) {
        System.out.println("started parsing file");
        try {
            File file = new File(filePath);
            BufferedReader reader = new BufferedReader(new FileReader(file));
            System.out.println("started reading lines");
            reader.lines().forEach(t -> {
                // lambda fuckery to get errors handled
                try {
                    parseLine(t);
                } catch (Exception e) {
                    // TODO Auto-generated catch block
                    System.out.println("Error reading line");
                    e.printStackTrace();
                }
            });
            reader.close();
        } catch (Exception e) {
            // if the file
            System.err.println("Invalid Auto File: " + e.getLocalizedMessage());
            e.printStackTrace();
        }

        SequentialCommandGroup allCommands = new SequentialCommandGroup();
        // allCommands.addCommands(new ResetAutoPos(Constants.START_POSE));
        SequentialCommandGroup entryCommands = new SequentialCommandGroup();
        for (AutoEntry entry : entries) {
            entryCommands.addCommands(entry.toCommand());
        }
        ParallelRaceGroup raceGroup = new ParallelRaceGroup(new WaitCommand(15), entryCommands);
        allCommands.addCommands(raceGroup);
        allCommands.addCommands((new IntakeOff("End", new ArrayList<Object>())).toCommand());
        allCommands.addCommands(new GoToArmPosition(Position.TRAVEL, RobotContainer.INSTANCE.arm, RobotContainer.INSTANCE.elevator));
        ArrayList<Object> positionParam = new ArrayList<>();
        positionParam.add("start");
        try {
            allCommands.addCommands((new GoToPosition("End", positionParam)).toCommand());
        } catch (Exception e) {}
        
        return allCommands;
    }

    public static void parseLine(String line) throws Exception {
        System.out.println("line parsed");
        if (line.startsWith("#")) return;
        line = line.strip();
        line = line.toLowerCase();
        
        String entryName = line.split(" ", 1)[0];
        String[] allParameters = line.split(" ", 2);
        if (allParameters.length > 1) {
            line = allParameters[1];
        } else {
            line = "";
        }
        String[] paramStrings = line.split(" ");
        ArrayList<Object> params = new ArrayList<>();
        for (String param : paramStrings) {
            if (param.equals("true")) {
                params.add(true);
                continue;
            }
            if (param.equals("false")) {
                params.add(false);
                continue;
            }
            boolean isNumber = false;
            try {
                double check = Double.parseDouble(param);
                isNumber = true;
                System.out.println(check);
            } catch (Exception e) {
                // Double.parseDouble fails if param does not have a number in it.
                isNumber = false;
            }
            if (isNumber) {
                params.add(Double.parseDouble(param));
                continue;
            }
            params.add(param);
            continue;
        }
        if (entryName.contains("arm_position")) {
            entries.add(new SetArmPositionEntry(entryName, params));
            return;
        }
        if (entryName.contains("print")) {
            entries.add(new DebugEntry(entryName, params));
            return;
        }
        if (entryName.contains("wait")) {
            entries.add(new WaitEntry(entryName, params));
            return;
        }
        if (entryName.contains("intake_off")) {
            entries.add(new IntakeOff(entryName, params));
            return;
        }
        if (entryName.contains("intake_on")) {
            entries.add(new IntakeOn(entryName, params));
            return;
        }
        if (entryName.contains("shoot")) {
            entries.add(new Shoot(entryName, params));
            return;
        }
        if (entryName.contains("go_to_position")) {
            entries.add(new GoToPosition(entryName, params));
            return;
        }
        System.out.println("Invalid Auto Line: \"" + line + "\", ignoring");
    }
}