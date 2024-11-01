package org.firstinspires.ftc.teamcode.c_subsystems

import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.SubsystemBase
import org.firstinspires.ftc.teamcode._configs.MecanumConfig

/**
 * A subsystem utilizing the [MecanumConfig] class. Handles robot movement and localization.
 */
class MecanumSubsystem
/**
 * Constructor for MecanumSubsystem.
 *
 * @param drive The MecanumDrive object controlling robot movement.
 */(internal val drive: MecanumConfig) : SubsystemBase() {
    /**
     * Updates the robot's pose estimate based on its odometry readings.
     */
    fun updatePoseEstimate() {
        drive.updatePoseEstimate()
    }

    /**
     * Drives the robot using the given joystick inputs.
     *
     * @param X The left joystick X input (strafe).
     * @param Y The left joystick Y input (straight).
     * @param A The right joystick X input (rotation).
     */
    fun setDrivePowers(X: Double, Y: Double, A: Double) {
        drive.setDrivePowers(
            PoseVelocity2d(Vector2d(X, Y), A)
        )
    }

    fun stopDrive() {
        setDrivePowers(0.0, 0.0, 0.0)
    }
}