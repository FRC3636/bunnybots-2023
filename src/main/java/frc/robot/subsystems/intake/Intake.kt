package frc.robot.subsystems.intake

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.RobotBase
import frc.robot.CANDevice
import frc.robot.subsystems.drivetrain.Drivetrain
import frc.robot.utils.PIDCoefficients
import frc.robot.utils.PIDController

object Intake : subsystem {

    val pidController = PIDController(PIDCoefficients(0.0,0.0,0.0))

    val feedForward = SimpleMotorFeedforward(0.0,0.0,0.0)

    private val io = {
        IntakeIOReal(CANDevice.IntakeFeedMotor, CANDevice.IntakeAngleMotor)
    }

    private val intakeInputs = IntakeInputs()

    // factoring in rotation of drivetrain
    private var targetRotation: Rotation2d = Rotation2d()


    override fun periodic() {

    }
    private val io = IntakeIOReal(CANDevice.IntakeFeedMotor, CANDevice.IntakeAngleMotor)
    fun takein() {
        io.setSpeed(
            pidController.calculate(relativeAngle.radians, targetRotation.radians)
                    + feedForward.calculate(Drivetrain.velocity2d.norm)
        )

    }




}


}