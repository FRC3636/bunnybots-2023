package frc.robot.subsystems.indexer

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.CANDevice
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface IndexerIO {
    class Inputs : LoggableInputs {

        var indexerMotorSpeed: Rotation2d = Rotation2d()
        var beamBroken: Boolean = false
        override fun toLog(table: LogTable?) {
            table?.put("Indexer Speed", indexerMotorSpeed.radians)
            table?.put("Beam Broken", beamBroken)
        }

        override fun fromLog(table: LogTable?) {
            table?.getDouble("Indexer Speed", indexerMotorSpeed.radians)
                    ?.let { indexerMotorSpeed = Rotation2d(it) }
            table?.getBoolean("Beam Broken", beamBroken)
        }
    }

    fun updateInputs(inputs: Inputs)

    fun setIndexerSpeed(speed: Double)
}

class IndexerIOReal(indexerMotorCAN: CANDevice) : IndexerIO {
    private val indexerMotor = CANSparkMax(indexerMotorCAN.id, CANSparkMaxLowLevel.MotorType.kBrushless)
    private var beamBreak = DigitalInput(BEAM_BREAK_DIO_PORT)

    override fun updateInputs(inputs: IndexerIO.Inputs) {
        inputs.indexerMotorSpeed = Rotation2d(indexerMotor.get())
        inputs.beamBroken = !beamBreak.get()
    }

    override fun setIndexerSpeed(speed: Double) {
        indexerMotor.set(speed)
    }

    internal companion object Constants {
        const val BEAM_BREAK_DIO_PORT = 1
    }
}