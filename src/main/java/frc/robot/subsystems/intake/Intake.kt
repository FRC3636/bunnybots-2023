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


    private val mechanism = Mechanism2d(3.0, 3.0)
    private val mechanismRoot: MechanismRoot2d = mechanism.getRoot("RobotConnection", 0.0, 1.5)
    private val mechanismMeasuredPosition = mechanismRoot.append(
        MechanismLigament2d("MeasuredPosition", 2.0, 90.0, 6.0, Color8Bit(Color.kWhite))
    )
    private val mechanismDesiredPosition = mechanismRoot.append(
        MechanismLigament2d("DesiredPosition", 2.0, 90.0, 6.0, Color8Bit(Color.kGreen))
    )

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.getInstance().processInputs(name, inputs)


        mechanismMeasuredPosition.angle = inputs.position.degrees
        Logger.getInstance().recordOutput("$name/Mechanism", mechanism)
    }

    fun runRollers(speed: Double) {
        io.setRollerSpeed(speed)
    }

    fun moveIntake(position: Rotation2d, velocity: Rotation2d) {
        val newVelocity = Rotation2d.fromRadians(velocity.radians +
                pidController.calculate(inputs.position.radians, max(
                    position.radians, 0.0
                )
                ))

        val voltage = feedForward.calculate(
            inputs.position.radians,
            newVelocity.radians,
        )

        io.setArmVoltage(voltage)
        mechanismDesiredPosition.angle = position.degrees

        Logger.getInstance().recordOutput("$name/DesiredPosition", position.degrees)
    }

    fun generateProfile(position: Rotation2d): TrapezoidProfile {
        return TrapezoidProfile(
            TrapezoidProfile.Constraints(0.0, 0.0),
            TrapezoidProfile.State(position.radians, 0.0),
            TrapezoidProfile.State(inputs.position.radians, inputs.armVelocity.radians)
        )
    }

}

