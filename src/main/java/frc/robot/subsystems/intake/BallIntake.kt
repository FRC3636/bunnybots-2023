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

    val pidController = PIDController(PIDCoefficients(0.0,0.0,0.0))

    val feedForward = ArmFeedforward(0.0,0.0,0.0)

    private val io: IntakeIO = IntakeIOReal(CANDevice.BallIntakeArmMotor.id, CANDevice.BallIntakeRollerMotor.id)

    private val inputs = IntakeIO.Inputs()

    override fun periodic() {
        Logger.getInstance().processInputs("Intake", inputs)

        io.updateInputs(inputs)
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
            inputs.position.radians - PI / 2,
            newVelocity.radians,
        )

        io.setArmVoltage(voltage)
    }

    fun generateProfile(position: Double): TrapezoidProfile {
        return TrapezoidProfile(
            TrapezoidProfile.Constraints(0.0, 0.0),
            TrapezoidProfile.State(position, 0.0),
            TrapezoidProfile.State(inputs.position.radians, inputs.velocity.radians)
        )
    }

    enum class Position(val setpoint: Double) {
        Up(0.0),
        Down(0.0)
    }

}


