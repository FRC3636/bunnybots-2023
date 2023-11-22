package frc.robot.subsystems.indexer

import edu.wpi.first.wpilibj2.command.*
import frc.robot.CANDevice
import org.littletonrobotics.junction.Logger

object Indexer : Subsystem {
    private val io = IndexerIOReal(CANDevice.IndexerMotor)
    private val inputs = IndexerIO.Inputs()

    /** Beam is broken */
    val objectDetected: Boolean
        get() {
            return inputs.beamBroken
        }


    val indexCommand: Command
        get(){
            return Commands.sequence(
                InstantCommand({ this.setSpeed(1.0)}),
                WaitCommand(7.0),
                InstantCommand({ this.setSpeed(0.0)})
            )
        }


    override fun periodic() {
        io.updateInputs(inputs)
        Logger.getInstance().processInputs("Indexer", inputs)
    }

    fun setSpeed(speed: Double) {
        io.setIndexerSpeed(speed)
    }
}