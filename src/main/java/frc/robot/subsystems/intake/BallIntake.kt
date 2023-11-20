package frc.robot.subsystems.intake

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.utils.PIDCoefficients
import frc.robot.utils.PIDController
import frc.robot.CANDevice
import org.littletonrobotics.junction.Logger
import kotlin.math.PI
import kotlin.math.max

object BallIntake : Subsystem {

    private val pidController = PIDController(PIDCoefficients())

    private val feedForward = ArmFeedforward(0.0,0.0,0.0)

    private val io: IntakeIO = IntakeIOReal(CANDevice.BallIntakeArmMotor.id, CANDevice.BallIntakeRollerMotor.id)

    private val inputs = IntakeIO.Inputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.getInstance().processInputs("Intake", inputs)
    }

    fun runRollers(speed: Double) {
        io.setRollerSpeed(speed)
    }

    fun moveIntake(position: Rotation2d, velocity: Rotation2d) {
        val newVelocity = Rotation2d.fromRadians(velocity.radians +
            pidController.calculate(inputs.position.radians, max(
                position.radians, 0.0
            )))

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

    enum class Position(val setpoint: Double) {
        Up(0.0),
        Down(0.0)
    }

}


