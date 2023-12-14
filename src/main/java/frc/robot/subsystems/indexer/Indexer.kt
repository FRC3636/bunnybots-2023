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

    fun autoIndexCommand(): Command{
        return SequentialCommandGroup(
            InstantCommand({ this.setSpeed(1.0) }),
            WaitCommand(2.0),
            InstantCommand({ this.setSpeed(0.0) }
        )).also{it.addRequirements(Indexer)}
    }

    val manualIndexCommand: Command
     get() {
        return this.startEnd({
            this.setSpeed(1.0)
        }, {
            this.setSpeed(0.0)
        })
    }


    override fun periodic() {
        io.updateInputs(inputs)
        Logger.getInstance().processInputs("Indexer", inputs)
    }

    fun setSpeed(speed: Double) {
        io.setIndexerSpeed(speed)
    }

    fun setSpeedCommand(speed: Double): Command {
        return run {
            setSpeed(speed)
        }
    }
}