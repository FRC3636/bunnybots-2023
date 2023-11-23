package frc.robot.commands

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.intake.Intake
import kotlin.math.atan2

class ControlIntakeWithJoystick(private val intake: Intake, private val getX: () -> Double, private val getY: () -> Double) : Command {
    override fun getRequirements() = setOf(intake)

    override fun execute() {
        val angle = atan2(getY(), getX())
        println("Intake to ${Units.radiansToDegrees(angle)}deg")
        intake.moveIntake(Rotation2d(angle), Rotation2d(2.0))
    }
}
