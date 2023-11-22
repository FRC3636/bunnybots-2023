package frc.robot.subsystems.turret

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.CANDevice
import frc.robot.subsystems.drivetrain.Drivetrain
import frc.robot.subsystems.targetvision.TargetVision
import frc.robot.utils.PIDCoefficients
import frc.robot.utils.PIDController
import org.littletonrobotics.junction.Logger
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.sign


object Turret : Subsystem {

    private val io = if (RobotBase.isReal()) {
        TurretIOReal(CANDevice.TurretMotor)
    } else {
        println("Using simulated turret")
        TurretIOSim()
    }
    private val inputs = TurretIO.Inputs()

    private val pidController = PIDController(PIDCoefficients(2.0, 0.0, 0.33))

    private val feedForward = SimpleMotorFeedforward(1.0, 0.0, 0.0)

    private val mechanism = Mechanism2d(3.0, 3.0)
    private val mechanismRoot: MechanismRoot2d = mechanism.getRoot("climber", 1.5, 1.5)
    private val mechanismAim = mechanismRoot.append(MechanismLigament2d("aim", 3.0, inputs.angle.degrees))

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

        io.updateInputs(inputs)
        mechanismAim.angle = inputs.angle.degrees
        Logger.getInstance().recordOutput("Turret/Mechanism", mechanism)
        Logger.getInstance().processInputs("Turret", inputs)



        val voltage = pidController.calculate(
            angleToChassis.radians, targetAngleToChassis.radians
        ) + feedForward.calculate(-Drivetrain.chassisSpeeds.omegaRadiansPerSecond)

        Logger.getInstance().recordOutput("Turret/Voltage", voltage)
        Logger.getInstance().recordOutput("Turret/TargetAngle", targetAngleToChassis.degrees)

        io.setVoltage(
           voltage
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
    fun setTargetCommand(setpoint: Rotation2d): Command{
        return InstantCommand({
            println("moving turret to $setpoint")
            this.setTarget(setpoint)
        })
    }



    /**
     * Aligns the turret to point the same direction as the joystick is being leaned.
     */
    fun controlWithJoysticks(joystickX: () -> Double, joystickY: () -> Double): Command {
        return run {
            val angle = atan2(joystickY(), joystickX())
            setTarget(Rotation2d(angle))
        }
    }

    /**
     * Aligns the turret to face the primary target.
     */
    fun trackPrimaryTarget(): Command {
        return run {
            if (TargetVision.hasTargets) {
                setTarget(angleToChassis + Rotation2d.fromDegrees(TargetVision.primaryTarget.tx))
            }
        }
    }
}
