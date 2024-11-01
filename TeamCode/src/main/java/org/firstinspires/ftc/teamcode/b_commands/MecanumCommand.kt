package org.firstinspires.ftc.teamcode.b_commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.c_subsystems.MecanumSubsystem
import java.util.function.DoubleSupplier

class MecanumCommand(
    private val drive: MecanumSubsystem?,
    private val forwardInput: DoubleSupplier,
    private val strafeInput: DoubleSupplier,
    private val turnInput: DoubleSupplier
) : CommandBase() {
    private var multiplier = 1.0

    init {
        addRequirements(drive)
    }

    /**
     * Constructor for MecanumCommand with a custom multiplier.
     *
     * @param drive      The drive subsystem this command will run on.
     * @param forward    The control input for driving forwards/backwards.
     * @param strafe     The control input for driving left/right.
     * @param turn       The control input for turning.
     * @param multiplier A multiplier for robot speed.
     */
    constructor(
        drive: MecanumSubsystem,
        forward: DoubleSupplier,
        strafe: DoubleSupplier,
        turn: DoubleSupplier,
        multiplier: Double
    ) : this(drive, forward, strafe, turn) {
        this.multiplier = multiplier
    }

    override fun execute() {
        val (strafeValue, forwardValue) = applyDeadzone(
            strafeInput.asDouble * 0.9 * multiplier, -forwardInput.asDouble * 0.9 * multiplier, 0.1
        )
        val (turnValue, _) = applyDeadzone(turnInput.asDouble * 0.8 * multiplier, 0.0, 0.1)

        drive?.setDrivePowers(forwardValue, strafeValue, turnValue)
    }

    private fun applyDeadzone(
        inputX: Double, inputY: Double, deadzone: Double
    ): Pair<Double, Double> {
        var d2 = Math.pow(inputX, 2.0) + Math.pow(inputY, 2.0)

        if (d2 < Math.pow(deadzone, 2.0) || d2 < 0) {
            return Pair(0.0, 0.0)
        } else {
            var d = Math.sqrt(d2)

            var nx = inputX / d
            var ny = inputY / d

            d = (d - deadzone) / (1 - deadzone)

            d = Math.min(d, 1.0)

            d = Math.pow(d, 2.0)

            return Pair(nx * d, ny * d)
        }
    }
}