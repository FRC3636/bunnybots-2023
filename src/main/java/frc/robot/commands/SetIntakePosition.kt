package frc.robot.commands

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.intake.Intake

class SetIntakePosition(private val setpoint: Rotation2d, private val intake: Intake) : Command {

    private lateinit var profile: TrapezoidProfile
    private var timer: Timer = Timer()

    override fun getRequirements() = setOf(intake)

    override fun initialize() {
        timer.reset()
        timer.start()
        profile = intake.generateProfile(setpoint)
    }

    override fun execute() {
        val state = profile.calculate(timer.get())
        intake.moveIntake(Rotation2d.fromRadians(state.position), Rotation2d.fromRadians(state.velocity))
    }

    override fun isFinished(): Boolean {
        return profile.isFinished(timer.get())
    }

}