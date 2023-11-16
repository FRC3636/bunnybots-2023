package frc.robot.subsystems.turret

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import frc.robot.CANDevice
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface TurretIO {
    class Inputs : LoggableInputs {
        var angle = Rotation2d()
        var angularVelocityHz = Rotation2d()

        override fun toLog(table: LogTable?) {
            table?.put("Angle", angle.radians)
            table?.put("Angular Velocity", angularVelocityHz.radians)
        }

        override fun fromLog(table: LogTable?) {
            table?.getDouble("Angle", 0.0)?.let { angle = Rotation2d.fromRadians(it) }
            table?.getDouble("Angular Velocity", 0.0)?.let { angle = Rotation2d.fromRadians(it) }
        }

    }

    fun updateInputs(inputs: Inputs)

    fun setVoltage(outputVolts: Double)
}


class TurretIOReal(motorCAN: CANDevice) : TurretIO {


    private val motor = CANSparkMax(motorCAN.id, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)


    init {
        encoder.positionConversionFactor = Units.rotationsToRadians(1.0) * GEAR_RATIO
        encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * GEAR_RATIO / 60
        motor.encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * GEAR_RATIO / 60
        motor.encoder.positionConversionFactor = Units.rotationsToRadians(1.0) * GEAR_RATIO / 60
    }

    override fun updateInputs(inputs: TurretIO.Inputs) {
        inputs.angle = Rotation2d(encoder.position)
        inputs.angularVelocityHz = Rotation2d(encoder.velocity)
    }

    override fun setVoltage(outputVolts: Double) {
        motor.setVoltage(outputVolts)
    }

    internal companion object Constants {
        // TODO: find turret gear ratio :0
        const val GEAR_RATIO = 1.0
    }
}

// TODO: implement sim turret

class TurretIOSim : TurretIO {
    override fun updateInputs(inputs: TurretIO.Inputs) {}
    override fun setVoltage(outputVolts: Double) {}
}