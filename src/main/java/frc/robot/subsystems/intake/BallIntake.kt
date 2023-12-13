package frc.robot.subsystems.intake

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.utils.PIDCoefficients
import frc.robot.utils.PIDController
object BallIntake : Intake() {

    override val pidController = PIDController(PIDCoefficients(25.0, 0.0, 0.0))

    override val feedForward = ArmFeedforward(0.0,0.462,1.0)

    override val io: IntakeIO = BallIntakeIOReal()

    override val inputs = IntakeIO.Inputs()

    override val name = "Ball"

    enum class Position(val pose: Rotation2d){
        Up(Rotation2d()),
        Down(Rotation2d(63.0)), // source: jackson
        Stowed(Rotation2d())
    }

}


