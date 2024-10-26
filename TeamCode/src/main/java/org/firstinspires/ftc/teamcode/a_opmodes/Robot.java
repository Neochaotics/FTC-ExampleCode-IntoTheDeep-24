package org.firstinspires.ftc.teamcode.a_opmodes;


/*
 * TYPE			NAME			ID		DESCRIPTION
 * ------------------------------------------------------------
 * MOTOR		frontLeft		fL		    Front Left Mecanum
 * MOTOR		frontRight		fR          Front Right Mecanum
 * MOTOR		backLeft		bL		    Back Left Mecanum
 * MOTOR		backRight		bR		    Back Right Mecanum
 * MOTOR        lift            lift        The Lift
 */

//@Config

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.util.List;

/**
 * This class represents the robot configuration and provides methods to initialize and control its components.
 */
public class Robot {
    // Hardware components
    public MotorEx frontLeft, backLeft, frontRight, backRight, lift;
    public VoltageSensor    voltageSensor;
    public FtcDashboard     dashboard = FtcDashboard.getInstance();
    public List<LynxModule> revHubs;

    // INSTANT COMMANDS
    public InstantCommand   INTAKE_TOGGLE, INTAKE_REVERSE ;


    // public IntakeSubsystem intakeSubsystem;
    // public DriveSubsystem driveSubsystem;

    /**
     * Constructor to initialize the robot hardware components.
     *
     * @param hardwareMap The hardware map containing all the robot components.
     * @param isAuto      A flag indicating whether the robot is in autonomous mode.
     */
    public Robot(HardwareMap hardwareMap, boolean isAuto) {
        configureRobot(hardwareMap, isAuto);

        // Initialize sensors and dashboard
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        dashboard = FtcDashboard.getInstance();

        // Bulk Read for REV Hubs
        revHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : revHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    /**
     * Constructor for the Robot class.
     *
     * @param hardwareMap The hardware map for accessing robot components.
     */
    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }

    /**
     * Configures the robot's hardware components and subsystems.
     *
     * @param hardwareMap The hardware map for accessing robot components.
     * @param isAuto      A flag indicating if the robot is in autonomous mode.
     */
    private void configureRobot(HardwareMap hardwareMap, boolean isAuto) {
        // MOTORS   ------------------------------------------------------------------------------------------------
        lift = new MotorEx(hardwareMap, "lift", MotorEx.GoBILDA.RPM_312);
        lift.resetEncoder();
        lift.setRunMode(MotorEx.RunMode.VelocityControl);
        // Set zero power behavior to BRAKE when no power is applied
        lift.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        // AUTO CONFIG  ------------------------------------------------------------------------------------------------
        if (isAuto) {
            // Add autonomous-specific configuration here
        }
    }

    /**
     * Retrieves and returns the current voltage of the robot.
     *
     * @param unit The unit in which the voltage is measured (e.g., VoltageUnit.VOLTS).
     * @return The current voltage of the robot in the specified unit.
     */
    public double getVoltage(VoltageUnit unit) {
        return revHubs.get(0).getInputVoltage(unit);
    }

    /**
     * Calculates and returns the total current draw of the robot.
     *
     * @param unit The unit in which the current is measured (e.g., CurrentUnit.AMPS).
     * @return The total current draw of the robot in the specified unit.
     */
    public double getCurrent(CurrentUnit unit) {
        double totalCurrent = 0;
        for (LynxModule hub : revHubs) {
            totalCurrent += hub.getCurrent(unit);
        }
        return totalCurrent;
    }
}
