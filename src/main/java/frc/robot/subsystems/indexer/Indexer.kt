package frc.robot.subsystems.indexer

import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.CANDevice
import org.littletonrobotics.junction.Logger

object Indexer : Subsystem {
    private val io = IndexerIOReal(CANDevice.IndexerMotor)
    private val inputs = IndexerIO.Inputs()

    val beamUnbroken: Boolean
        get() {
            return inputs.beamUnbroken
        }


    override fun periodic() {
        io.updateInputs(inputs)
        Logger.getInstance().processInputs("Indexer", inputs)
    }

    fun setSpeed(speed: Double) {
        io.setIndexerSpeed(speed)
    }
}