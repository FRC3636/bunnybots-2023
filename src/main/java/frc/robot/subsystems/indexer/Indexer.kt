package frc.robot.subsystems.indexer

import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.CANDevice

class Indexer : Subsystem {
    private val io = IndexerIOReal(CANDevice.IndexerMotor)
    private val inputs = IndexerIO.IndexerInputs()
    override fun periodic() {
        io.updateInputs(inputs)
    }

    fun index(speed: Double) {
        io.setIndexerSpeed(speed)
    }
}