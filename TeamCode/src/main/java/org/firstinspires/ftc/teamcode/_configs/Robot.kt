package org.firstinspires.ftc.teamcode._configs

import com.acmerobotics.dashboard.FtcDashboard
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit

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


/**
 * This class represents the robot configuration and provides methods to initialize and control its
 * components.
 */
class Robot @JvmOverloads constructor(hardwareMap: HardwareMap, isAuto: Boolean = false) {
    // Hardware components
    var frontLeft: MotorEx? = null
    var backLeft: MotorEx? = null
    var frontRight: MotorEx? = null
    var backRight: MotorEx? = null
    var lift: MotorEx? = null
    var voltageSensor: VoltageSensor
    var dashboard: FtcDashboard = FtcDashboard.getInstance()
    var revHubs: List<LynxModule>

    // INSTANT COMMANDS
    var INTAKE_TOGGLE: InstantCommand? = null
    var INTAKE_REVERSE: InstantCommand? = null


    // public IntakeSubsystem intakeSubsystem;
    // public DriveSubsystem driveSubsystem;
    /**
     * Constructor to initialize the robot hardware components.
     *
     * @param hardwareMap The hardware map containing all the robot components.
     * @param isAuto      A flag indicating whether the robot is in autonomous mode.
     */
    /**
     * Constructor for the Robot class.
     *
     * @param hardwareMap The hardware map for accessing robot components.
     */
    init {
        configureRobot(hardwareMap, isAuto)

        // Initialize sensors and dashboard
        voltageSensor = hardwareMap.voltageSensor.iterator().next()
        dashboard = FtcDashboard.getInstance()

        // Bulk Read for REV Hubs
        revHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in revHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
    }

    /**
     * Configures the robot's hardware components and subsystems.
     *
     * @param hardwareMap The hardware map for accessing robot components.
     * @param isAuto      A flag indicating if the robot is in autonomous mode.
     */
    private fun configureRobot(hardwareMap: HardwareMap, isAuto: Boolean) {
        // MOTORS   ------------------------------------------------------------------------------------------------
        lift = MotorEx(hardwareMap, "lift", Motor.GoBILDA.RPM_312)
        lift!!.resetEncoder()
        lift!!.setRunMode(Motor.RunMode.VelocityControl)
        // Set zero power behavior to BRAKE when no power is applied
        lift!!.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)

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
    fun getVoltage(unit: VoltageUnit?): Double {
        return revHubs[0].getInputVoltage(unit)
    }

    /**
     * Calculates and returns the total current draw of the robot.
     *
     * @param unit The unit in which the current is measured (e.g., CurrentUnit.AMPS).
     * @return The total current draw of the robot in the specified unit.
     */
    fun getCurrent(unit: CurrentUnit?): Double {
        var totalCurrent = 0.0
        for (hub in revHubs) {
            totalCurrent += hub.getCurrent(unit)
        }
        return totalCurrent
    }
}
