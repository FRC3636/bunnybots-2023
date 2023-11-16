package frc.robot.subsystems.indexer

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.CANDevice
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface IndexerIO {
    class IndexerInputs : LoggableInputs {

        var indexerMotorSpeed: Rotation2d = Rotation2d()
        override fun toLog(table: LogTable?) {
            table?.put("Indexer Speed", indexerMotorSpeed.radians)
        }

        override fun fromLog(table: LogTable?) {
            table?.getDouble("Indexer Speed", indexerMotorSpeed.radians)
                    ?.let { indexerMotorSpeed = Rotation2d(it) }
        }

    }

    fun updateInputs(inputs: IndexerInputs)

    fun setIndexerSpeed(speed: Double)
}

class IndexerIOReal(indexerMotorCAN: CANDevice) : IndexerIO {
    private val indexerMotor = CANSparkMax(indexerMotorCAN.id, CANSparkMaxLowLevel.MotorType.kBrushless)

    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        inputs.indexerMotorSpeed = Rotation2d(indexerMotor.get())
    }

    override fun setIndexerSpeed(speed: Double) {
        indexerMotor.set(speed)
    }
}