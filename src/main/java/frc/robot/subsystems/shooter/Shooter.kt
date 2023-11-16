package frc.robot.subsystems.shooter

import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.CANDevice
import org.littletonrobotics.junction.Logger


object Shooter : Subsystem {

    // TODO: Implement feedforward maybe


    private val inputs = ShooterIO.ShooterInputs()

    private val io = ShooterIOReal(CANDevice.ShooterWheelMotor, CANDevice.ShooterFeedMotor)

    override fun periodic() {

        io.updateInputs(inputs)

        Logger.getInstance().recordOutput("Shooter/YeetMainMotorVelocity", io.getSpeedMain())
        Logger.getInstance().recordOutput("Shooter/YeetSecondaryMotorVelocity", io.getSpeedSecondary())
    }

    fun spin(speed: Double) {
        io.setSpeedMain(speed)
    }

    fun feed(speed: Double){
        io.setSpeedSecondary(speed)
    }
}