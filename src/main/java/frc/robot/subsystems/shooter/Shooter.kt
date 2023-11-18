package frc.robot.subsystems.shooter

import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.CANDevice
import org.littletonrobotics.junction.Logger


object Shooter : Subsystem {

    // TODO: Implement feedforward maybe


    private val inputs = ShooterIO.ShooterInputs()

    private val io = ShooterIOReal(CANDevice.FlywheelMotor.id, CANDevice.ShooterFeedMotor.id)


    override fun periodic() {
        io.updateInputs(inputs)
    }

    fun spin(speed: Double) {
        io.setSpeedFlywheel(speed)
    }

    fun feed(speed: Double){
        io.setSpeedFeeder(speed)
    }


}