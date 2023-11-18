package frc.robot.subsystems.indexer

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.CANDevice
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface IndexerIO {
    class IndexerInputs : LoggableInputs {

        var indexerMotorSpeed: Rotation2d = Rotation2d()
        var beamBreakStatus: Boolean = true
        override fun toLog(table: LogTable?) {
            table?.put("Indexer Speed", indexerMotorSpeed.radians)
            table?.put("Beam Break", beamBreakStatus)
        }

        override fun fromLog(table: LogTable?) {
            table?.getDouble("Indexer Speed", indexerMotorSpeed.radians)
                    ?.let { indexerMotorSpeed = Rotation2d(it) }
            table?.getBoolean("Beam Break", beamBreakStatus)
        }

    }

    fun updateInputs(inputs: IndexerInputs)

    fun setIndexerSpeed(speed: Double)
}

class IndexerIOReal(indexerMotorCAN: CANDevice) : IndexerIO {
    private val indexerMotor = CANSparkMax(indexerMotorCAN.id, CANSparkMaxLowLevel.MotorType.kBrushless)
    private var beamBreak = DigitalInput(BEAM_BREAK)

    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        inputs.indexerMotorSpeed = Rotation2d(indexerMotor.get())
        inputs.beamBreakStatus = beamBreak.get()
    }

    override fun setIndexerSpeed(speed: Double) {
        indexerMotor.set(speed)
    }

    internal companion object Constants {
        const val BEAM_BREAK = 1
    }
}