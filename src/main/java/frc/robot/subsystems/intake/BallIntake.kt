package frc.robot.subsystems.intake

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.utils.PIDCoefficients
import frc.robot.utils.PIDController
import org.littletonrobotics.junction.Logger

object BallIntake : Intake() {

    override val pidController = PIDController(PIDCoefficients(2.5, 0.0, 0.06))

    override val feedForward = ArmFeedforward(0.0,-0.15,-2.55)

    override val io: IntakeIO = BallIntakeIOReal()


    override val inputs = IntakeIO.Inputs()

    override val name = "Ball"


    enum class Position(val pose: Rotation2d){
        Up(Rotation2d(1.45)),
        Down(Rotation2d(0.45)), // source: jackson
        Stowed(Rotation2d())
    }

    init {

        CommandScheduler.getInstance().registerSubsystem(this)
    }

}


