package frc.robot;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Settings {
    private CommandXboxController driveController, operatorController;
    public OperatorSettings operatorSettings;
    public DriverSettings driverSettings;

    public Settings (CommandXboxController driveController, CommandXboxController operatorController) {
        this.operatorController = operatorController;
        this.driveController = driveController;

        operatorSettings = new OperatorSettings();
        driverSettings = new DriverSettings();
    }
    public class DriverSettings {
        public Trigger speedModeButton = new Trigger(()-> {return driveController.getRightTriggerAxis() > 0.5;});
        public Trigger invertButton = driveController.start();
        public boolean inverted = false;

        public double getLeftX() {
            double axis = driveController.getLeftX();
            axis = MathUtil.applyDeadband(axis, 0.1);
            if (inverted) {axis = -axis;}
            return axis;
        }
        public double getLeftY() {
            double axis = driveController.getLeftY();
            axis = MathUtil.applyDeadband(axis, 0.1);
            if (inverted) {axis = -axis;}
            return axis;
        }
        public double getRightX() {
            double axis = driveController.getRightX();
            axis = MathUtil.applyDeadband(axis, 0.1);
            if (inverted) {axis = -axis;}
            return axis;
        }
        public double getRightY() {
            double axis = driveController.getRightY();
            axis = MathUtil.applyDeadband(axis, 0.1);
            if (inverted) {axis = -axis;}
            return axis;
        }
    }
    /**Controller bindings and such for controlling arm and arm adjacent parts (eg:intake and elevator) */
    public class OperatorSettings {
        public Trigger coralIntakeButton       = operatorController.x();
        // public Trigger armAngleButton          = operatorController.b();
        public Trigger releaseButton    = operatorController.b();
        public Trigger travelButton = operatorController.y();
        public Trigger overrideArm = new Trigger(() -> {return Math.abs(operatorController.getRightY()) > 0.1;});

        public Trigger climbButton = operatorController.leftBumper();
        public Trigger unClimbButton = operatorController.rightBumper();

        public Trigger moveToL2 = operatorController.povDown();
        public Trigger moveToL3 = operatorController.povUp();
        public Trigger moveToIntake = operatorController.a();
        public Trigger moveToLoAlgae = operatorController.leftTrigger();
        public Trigger moveToHiAlgae = operatorController.rightTrigger();

        public final double intakeSpeed = -1/3.0;
        public final double releaseSpeed = 0.5;

        public double climbSpeed = 5;

        
        public double getLeftX() {
            double axis = operatorController.getLeftX();
            axis = MathUtil.applyDeadband(axis, 0.1);
            return axis;
        }
        public double getLeftY() {
            double axis = operatorController.getLeftY();
            axis = MathUtil.applyDeadband(axis, 0.1);
            return axis;
        }
        public double getRightX() {
            double axis = operatorController.getRightX();
            axis = MathUtil.applyDeadband(axis, 0.1);
            return axis;
        }
        public double getRightY() {
            double axis = operatorController.getRightY();
            axis = MathUtil.applyDeadband(axis, 0.1);
            return axis;
        }
    }
}
