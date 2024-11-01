package org.firstinspires.ftc.teamcode.a_opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode._configs.RobotConfig
import org.firstinspires.ftc.teamcode.b_commands.MecanumCommand


//@Disabled
@Config
@TeleOp(name = "MainTeleOp", group = ".Drive")
class MainTeleOp : CommandOpMode() {
    private var robot: RobotConfig? = null

    override fun initialize() {
        // Initialize the robot hardware
        robot = RobotConfig(hardwareMap)

        // Set up telemetry on both Driver Station and Dashboard
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        // Set up gamepad controls
        val gamepad1Ex = GamepadEx(gamepad1)
        val mecanumCommand = MecanumCommand(
            robot!!.mecanumSubsystem,
            { gamepad1Ex.leftY },
            { gamepad1Ex.leftX },
            { gamepad1Ex.rightX }
        )

        // Register and schedule commands
        gamepad1Ex.readButtons()
        schedule(mecanumCommand)
    }
}
