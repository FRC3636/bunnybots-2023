package frc.robot.subsystems.turret

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.CANDevice
import frc.robot.subsystems.drivetrain.Drivetrain
import frc.robot.utils.PIDCoefficients
import frc.robot.utils.PIDController
import kotlin.math.absoluteValue
import kotlin.math.sign


object Turret : Subsystem {

    private val io = if (RobotBase.isReal()) {
        TurretIOReal(CANDevice.TurretMotor)
    } else {
        TurretIOSim()
    }
    private val inputs = TurretIO.Inputs()

    private val pidController = PIDController(PIDCoefficients(0.0, 0.0, 0.0))

    private val feedForward = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    private var targetAngleToChassis: Rotation2d = Rotation2d()
        set(value) {
            var degrees = value.degrees % 360

            if (degrees.absoluteValue > 180) {
                degrees -= 360 * degrees.sign
            }
            degrees = degrees.coerceIn(-MAX_ROTATION_DEGREES, MAX_ROTATION_DEGREES)
            field = Rotation2d.fromDegrees(degrees)
        }


    override fun periodic() {
        io.setVoltage(
            pidController.calculate(
                angleToField.radians, targetAngleToChassis.radians
            ) + feedForward.calculate(-Drivetrain.chassisSpeeds.omegaRadiansPerSecond)
        )
    }

    fun setTarget(target: Rotation2d) {
        targetAngleToChassis = target
    }

    val angleToChassis: Rotation2d
        get() = inputs.angle

    val angleToField: Rotation2d
        get() = inputs.angle.plus(Drivetrain.estimatedPose.rotation)

    // Constants
    private const val MAX_ROTATION_DEGREES = 90.0
}
