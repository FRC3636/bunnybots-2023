package frc.robot.subsystems.intake

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.utils.PIDCoefficients
import frc.robot.utils.PIDController
import frc.robot.CANDevice

object BallIntake : Subsystem {

    val pidController = PIDController(PIDCoefficients(0.0,0.0,0.0))

    val feedForward = ArmFeedforward(0.0,0.0,0.0)


    private val io: IntakeIO =  IntakeIOReal(CANDevice.BallIntakeArmMotor.id, CANDevice.BallIntakeRollerMotor.id)

    private val inputs = IntakeIO.Inputs()

    private var targetRotation: Rotation2d = Rotation2d()
    override fun periodic() {
        io.updateInputs(inputs)

    }


    fun runRollers(speed: Double) {
        io.setRollerSpeed(speed)
    }






}


