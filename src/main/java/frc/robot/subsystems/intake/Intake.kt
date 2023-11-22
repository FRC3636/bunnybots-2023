package frc.robot.subsystems.intake

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import kotlin.math.max


abstract class Intake : Subsystem {

    abstract val pidController: PIDController

    abstract val feedForward: ArmFeedforward

    abstract val io: IntakeIO

    abstract val inputs: IntakeIO.Inputs

    abstract val name: String

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.getInstance().processInputs("($name)Intake", inputs)
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
    }

    fun generateProfile(position: Rotation2d): TrapezoidProfile {
        return TrapezoidProfile(
            TrapezoidProfile.Constraints(0.0, 0.0),
            TrapezoidProfile.State(position.radians, 0.0),
            TrapezoidProfile.State(inputs.position.radians, inputs.armVelocity.radians)
        )
    }

}

