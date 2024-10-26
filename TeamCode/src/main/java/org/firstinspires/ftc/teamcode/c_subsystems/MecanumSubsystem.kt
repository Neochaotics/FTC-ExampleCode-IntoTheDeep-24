package org.firstinspires.ftc.teamcode.c_subsystems;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.d_roadrunner.MecanumDrive;

/**
 * A subsystem utilizing the {@link MecanumDrive} class. Handles robot movement and localization.
 */
public class MecanumSubsystem extends SubsystemBase {

    private final MecanumDrive drive;
    private final boolean      isFieldCentric;

    /**
     * Constructor for MecanumSubsystem.
     *
     * @param drive          The MecanumDrive object controlling robot movement.
     * @param isFieldCentric A boolean indicating if the robot operates in field-centric mode.
     */
    public MecanumSubsystem(MecanumDrive drive, boolean isFieldCentric) {
        this.drive          = drive;
        this.isFieldCentric = isFieldCentric;
    }

    /**
     * Updates the robot's pose estimate based on its odometry readings.
     */
    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    /**
     * Drives the robot using the given joystick inputs.
     *
     * @param leftY  The left joystick Y input.
     * @param leftX  The left joystick X input.
     * @param rightX The right joystick X input.
     */
    public void drive(double leftY, double leftX, double rightX) {
        drive.setDrivePowers(
            new PoseVelocity2d(new Vector2d(leftX, leftY), rightX));
    }

}
