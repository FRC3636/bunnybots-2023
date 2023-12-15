package frc.robot.subsystems.turret

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.CANDevice
import frc.robot.subsystems.drivetrain.Drivetrain
import frc.robot.subsystems.targetvision.Limelight
import frc.robot.subsystems.targetvision.TargetVisionIO
import frc.robot.utils.LimelightHelpers
import frc.robot.utils.PIDCoefficients
import frc.robot.utils.PIDController
import org.littletonrobotics.junction.Logger
import kotlin.math.absoluteValue
import kotlin.math.sign


object Turret : Subsystem {


    private val io = if (RobotBase.isReal()) {
        TurretIOReal(CANDevice.TurretMotor)
    } else {
        TurretIOSim()
    }

    private val inputs = TurretIO.Inputs()

    private val pidController = PIDController(PIDCoefficients(10.00, 0.000, 0.1))

    private val mechanism = Mechanism2d(3.0, 3.0)
    private val mechanismRoot: MechanismRoot2d = mechanism.getRoot("climber", 1.5, 1.5)
    private val mechanismAim = mechanismRoot.append(MechanismLigament2d("aim", 3.0, inputs.angle.degrees / 5.0))

//    private var targetAngleToChassis: Rotation2d = Rotation2d()
//        set(value) {
//            var degrees = value.degrees % 360
//
//            if (degrees.absoluteValue > 180) {
//                degrees -= 360 * degrees.sign
//            }
//            degrees = degrees.coerceIn(-MAX_ROTATION_DEGREES, MAX_ROTATION_DEGREES)
//            field = Rotation2d.fromDegrees(degrees)
//        }

    private var speed = 0.0
    var mode = TurretMode.Manual
        set(value) {
            field = value
            when (value) {
                TurretMode.Follow -> {
                    LimelightHelpers.setCameraMode_Processor("limelight")
                    LimelightHelpers.setLEDMode_ForceOn("limelight")
                }
                else -> {
                    LimelightHelpers.setCameraMode_Driver("limelight")
                    LimelightHelpers.setLEDMode_ForceOff("limelight")
                }
            }
        }

    enum class TurretMode {
        Manual,
        Zero,
        Follow,
    }


    override fun periodic() {

        io.updateInputs(inputs)
        mechanismAim.angle = inputs.angle.degrees
        Logger.getInstance().recordOutput("Turret/Mechanism", mechanism)
        Logger.getInstance().processInputs("Turret", inputs)


//        val voltage = pidController.calculate(
//            angleToChassis.radians, targetAngleToChassis.radians
//        )
        // + feedForward.calculate(-Drivetrain.chassisSpeeds.omegaRadiansPerSecond)


        var volts = getVolts()
        val atOrPastMaxValue = inputs.angle.degrees.absoluteValue >= MAX_ROTATION_DEGREES
        val goingTowardsMaxValue = volts.sign == inputs.angle.degrees.sign
        if (atOrPastMaxValue && goingTowardsMaxValue)
        {
            println("reached max angle, ignoring turn instruction")
            volts = 0.0
        }
        Logger.getInstance().recordOutput("Turret/Voltage", volts)
        io.setVoltage(
            volts
        )

    }

    private fun getVolts(): Double {
        if (mode == TurretMode.Manual) {
            return speed
        }

        val targetPos = if (mode == TurretMode.Zero) {
            Rotation2d()
        } else {
            val inputs = TargetVisionIO.Inputs()
            Limelight.updateInputs(inputs)
            val targets = inputs.targets.filter {
                it.className == if (DriverStation.getAlliance() == Alliance.Blue) {
                    "red"
                } else {
                    "blue"
                }
            }

            if (targets.isEmpty()) {
                return 0.0
            }

            Logger.getInstance().recordOutput("Turret/tx", targets.first().tx)
           ( (Rotation2d.fromDegrees(targets.first().tx) + Rotation2d.fromDegrees(speed * -2.0)) * -1.0) + angleToChassis
        }

        return pidController.calculate(
            angleToChassis.radians, targetPos.radians
        )
    }

    fun setSpeed(volts: Double) {
        speed = volts
    }

    val angleToChassis: Rotation2d
        get() = inputs.angle

    val angleToField: Rotation2d
        get() = inputs.angle.plus(Drivetrain.estimatedPose.rotation)

    // Constants
    private const val MAX_ROTATION_DEGREES = 90.0
    fun setSpeedCommand(setpoint: Double): Command {
        return InstantCommand({
            this.setSpeed(setpoint)
        })
    }

    /**
     * Aligns the turret to point the same direction as the joystick is being leaned.
     */
//    fun controlWithJoysticks(joystickX: () -> Double, joystickY: () -> Double): Command {
//        return run {
//            val angle = atan2(joystickY(), joystickX())
//            setTarget(Rotation2d(angle))
//        }
//    }


    /**
     * Aligns the turret to face the primary target.
     */
//    fun trackPrimaryTarget(): Command {
//        return run {
//            if (TargetVision.hasTargets) {
//                val target = TargetVision.primaryTargetSamples.last().pose
//                setTarget(Rotation2d(atan2(target.y, target.x)))
//            }
//        }
//    }
}
