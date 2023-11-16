package frc.robot.subsystems.yeeter

import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.CANDevice
import frc.robot.subsystems.targetvision.TargetVision
import org.littletonrobotics.junction.Logger


object Yeeter : Subsystem {

    // TODO: implement feedforward maybe
    // TODO: this needs a sim impl

    private val io = YeeterIOReal(CANDevice.YeetWheelMotor, CANDevice.YeetFeedMotor)
    private val inputs = YeeterIO.Inputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.getInstance().processInputs("Yeeter", inputs)
    }

    fun yeet() {
        throw NotImplementedError()
    }
}