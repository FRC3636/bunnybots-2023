package frc.robot.commands

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.intake.BallIntake

class SetIntakePosition(private val setPosition: BallIntake.Position) : Command {

    private lateinit var profile: TrapezoidProfile
    private var timer: Timer = Timer()


    override fun getRequirements() = setOf(BallIntake)

    override fun initialize() {
        timer.reset()
        timer.start()
        profile = BallIntake.generateProfile(setPosition.setpoint)
    }

    override fun execute() {
        val state = profile.calculate(timer.get())

        BallIntake.moveIntake(Rotation2d.fromRadians(state.position), Rotation2d.fromRadians(state.velocity))
    }

    override fun isFinished(): Boolean {
        return profile.isFinished(timer.get())
    }

}