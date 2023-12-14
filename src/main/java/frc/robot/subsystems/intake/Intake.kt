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
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
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

    private var setpointPosition: Rotation2d? = null
    private var setpointVelocity: Rotation2d? = null

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.getInstance().processInputs(name, inputs)

        if (setpointPosition != null && setpointVelocity != null) {
            val newVelocity = Rotation2d.fromRadians(setpointVelocity!!.radians +
                    pidController.calculate(inputs.position.radians, setpointPosition!!.radians))

            val voltage = feedForward.calculate(
                inputs.position.radians,
                newVelocity.radians
            )
            io.setArmVoltage(voltage)

            Logger.getInstance().recordOutput("$name/voltageApplied", voltage)

            mechanismDesiredPosition.angle = setpointPosition!!.degrees

            Logger.getInstance().recordOutput("$name/DesiredPosition", setpointPosition!!.radians)
            Logger.getInstance().recordOutput("$name/NewVelocity", newVelocity.radians)
        }



        mechanismMeasuredPosition.angle = inputs.position.degrees
        Logger.getInstance().recordOutput("$name/Mechanism", mechanism)
    }

    fun runRollers(speed: Double) {
        io.setRollerSpeed(speed)
    }

    fun moveIntake(position: Rotation2d, velocity: Rotation2d) {
        setpointPosition = position
        setpointVelocity = velocity
    }

    fun runRollersCommand(speed: Double): Command {
        return runOnce {
            this.runRollers(speed);
        }
    }

    fun generateProfile(position: Rotation2d): TrapezoidProfile {
        return TrapezoidProfile(
            TrapezoidProfile.Constraints(15.0, 2.0),
            TrapezoidProfile.State(position.radians, 0.0),
            TrapezoidProfile.State(inputs.position.radians -0.02, inputs.armVelocity.radians)
        )
    }

}

