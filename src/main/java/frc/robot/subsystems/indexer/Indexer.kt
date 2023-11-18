package frc.robot.subsystems.indexer

import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.CANDevice

object Indexer : Subsystem {
    private val io = IndexerIOReal(CANDevice.IndexerMotor)
    private val inputs = IndexerIO.Inputs()

    val beamBreak: Boolean
        get() {
            return inputs.beamBreakStatus
        }


    override fun periodic() {
        io.updateInputs(inputs)
    }

    fun setSpeed(speed: Double) {
        io.setIndexerSpeed(speed)
    }
}