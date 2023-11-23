package frc.robot.subsystems.intake

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import kotlin.math.max


abstract class Intake : Subsystem {

    abstract val pidController: PIDController

    abstract val feedForward: ArmFeedforward

    abstract val io: IntakeIO

    abstract val inputs: IntakeIO.Inputs

    abstract val name: String
    
    companion object MechanismParts {
        private val mechanism = Mechanism2d(3.0, 3.0)
        private val root: MechanismRoot2d = mechanism.getRoot("RobotConnection", 0.25, 0.25)
        private val measuredPosition = root.append(
            MechanismLigament2d("MeasuredPosition", 2.0, 90.0, 6.0, Color8Bit(Color.kWhite))
        )
        private val desiredPosition = root.append(
            MechanismLigament2d("DesiredPosition", 2.0, 90.0, 6.0, Color8Bit(Color.kGreen))
        )
        private val desiredWheelSpokes = createWheelSpokes(desiredPosition, Color8Bit(Color.kDarkGreen))
        private var desiredWheelSpokesOffset = Rotation2d()
            set(value) {
                field = value
                desiredWheelSpokes.forEachIndexed { index, spoke ->
                    spoke.angle = (index * 90.0) - value.degrees
                }
            }
        private val measuredWheelSpokes = createWheelSpokes(measuredPosition, Color8Bit(Color.kGray))
        private var measuredWheelSpokesOffset = Rotation2d()
            set(value) {
                field = value
                measuredWheelSpokes.forEachIndexed { index, spoke ->
                    spoke.angle = (index * 90.0) - value.degrees
                }
            }

        private fun createWheelSpokes(parent: MechanismLigament2d, color: Color8Bit): Array<MechanismLigament2d> {
            val spokes = Array(4) { i ->
                MechanismLigament2d("Spoke$i", 0.25, i * 90.0, 6.0, color)
            }

            for (spoke in spokes) {
                parent.append(spoke)
            }

            return spokes
        }
    }

    private var rollersSpeed = 0.0

    override fun periodic() {
        val className = this.javaClass.simpleName
        io.updateInputs(inputs)
        Logger.getInstance().processInputs(className, inputs)


        measuredPosition.angle = inputs.position.degrees
        if (inputs.rollersOn) {
            measuredWheelSpokesOffset = measuredWheelSpokesOffset.plus(Rotation2d.fromDegrees(4.0))
        }
        if (rollersSpeed > 0.0) {
            desiredWheelSpokesOffset = desiredWheelSpokesOffset.plus(Rotation2d.fromDegrees(4.0))
        }
        Logger.getInstance().recordOutput("$className/Mechanism", mechanism)
    }

    fun runRollers(speed: Double) {
        rollersSpeed = speed
        io.setRollerSpeed(speed)
    }

    fun moveIntake(position: Rotation2d, velocity: Rotation2d) {
        val className = this.javaClass.simpleName
        val pidCalculations = pidController.calculate(inputs.position.radians, max(
            position.radians, 0.0
        ))
        Logger.getInstance().recordOutput("$className/PID", pidCalculations)

        val newVelocity = Rotation2d.fromRadians(velocity.radians +
                pidCalculations)

        val voltage = feedForward.calculate(
            inputs.position.radians,
            newVelocity.radians,
        )
        Logger.getInstance().recordOutput("$className/Voltage", voltage)

        io.setArmVoltage(voltage)
        desiredPosition.angle = position.degrees

        Logger.getInstance().recordOutput("$className/DesiredPosition", position.degrees)
        Logger.getInstance().recordOutput("$className/DesiredVelocity", velocity.degrees)
    }

    fun generateProfile(position: Rotation2d): TrapezoidProfile {
        return TrapezoidProfile(
            TrapezoidProfile.Constraints(0.0, 0.0),
            TrapezoidProfile.State(position.radians, 0.0),
            TrapezoidProfile.State(inputs.position.radians, inputs.armVelocity.radians)
        )
    }

}
