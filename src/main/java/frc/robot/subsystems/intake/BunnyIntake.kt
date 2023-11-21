package frc.robot.subsystems.intake

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.utils.PIDCoefficients
import frc.robot.utils.PIDController

object BunnyIntake : Intake() {

    override val pidController = PIDController(PIDCoefficients())

    override val feedForward = ArmFeedforward(0.0,0.0,0.0)

    override val io: IntakeIO = BallIntakeIOReal()

    override val inputs = IntakeIO.Inputs()

    override val name = "Bunny"

}