package frc.robot.subsystems.shooter

import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.CANDevice
import org.littletonrobotics.junction.Logger


object Shooter : Subsystem {

    // TODO: Implement feedforward maybe


    private val shooterInputs = ShooterInputs()

    val io = ShooterIOReal(CANDevice.ShooterMotorMain, CANDevice.ShooterMotorSecondary)

    override fun periodic() {

        io.updateInputs(shooterInputs)

        Logger.getInstance().recordOutput("Shooter/MainMotorVelocity", io.getSpeedMain())
        Logger.getInstance().recordOutput("Shooter/SecondaryMotorVelocity", io.getSpeedSecondary())
    }

    fun shoot() {
        throw NotImplementedError("Not implemented")
    }
}