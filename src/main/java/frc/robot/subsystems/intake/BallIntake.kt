package frc.robot.subsystems.intake

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.CANDevice
import frc.robot.subsystems.drivetrain.Drivetrain
import frc.robot.utils.PIDCoefficients
import frc.robot.utils.PIDController

object BallIntake : Subsystem {

    val pidController = PIDController(PIDCoefficients(0.0,0.0,0.0))

    val feedForward = ArmFeedforward(0.0,0.0,0.0)

    private val io: IntakeIO =  IntakeIOReal(CANDevice.IntakeFeedMotor, CANDevice.IntakeAngleMotor)

    private val intakeInputs = IntakeInputs()

    private var targetRotation: Rotation2d = Rotation2d()
    override fun periodic() {
        io.updateInputs()

    }


    fun takein() {
        io.setSpeed(
            pidController.calculate(relativeAngle.radians, targetRotation.radians)
                    + feedForward.calculate(Drivetrain.velocity2d.norm)
        )

    }




}


