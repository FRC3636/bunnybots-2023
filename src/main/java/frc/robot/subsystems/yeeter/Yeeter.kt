package frc.robot.subsystems.yeeter

import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.CANDevice
import org.littletonrobotics.junction.Logger


object Yeeter : Subsystem {

    // TODO: Implement feedforward maybe


    private val yeeterInputs = YeeterInputs()

    val io = YeeterIOReal(CANDevice.YeetWheelMotor, CANDevice.YeetFeedMotor)

    override fun periodic() {

        io.updateInputs(yeeterInputs)

        Logger.getInstance().recordOutput("Shooter/YeetMainMotorVelocity", io.getSpeedMain())
        Logger.getInstance().recordOutput("Shooter/YeetSecondaryMotorVelocity", io.getSpeedSecondary())
    }

    fun yeet() {
        throw NotImplementedError("Not implemented")
    }
}