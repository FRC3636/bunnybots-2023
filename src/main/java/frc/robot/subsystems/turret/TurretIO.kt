package frc.robot.subsystems.turret

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.simulation.DCMotorSim
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
        const val GEAR_RATIO = 5.0
    }
}


class TurretIOSim : TurretIO {
    companion object {
        // calculated from cad, but probably wrong
        private const val MOMENT_OF_INERTIA_M2_KG = 0.0103
    }
    private val sim = DCMotorSim(DCMotor.getNeo550(1), 5.0, MOMENT_OF_INERTIA_M2_KG)
    private var volts = 0.0

    override fun updateInputs(inputs: TurretIO.Inputs) {
        sim.update(0.02)
        inputs.angle = Rotation2d(sim.angularPositionRad)
        inputs.angularVelocityHz = Rotation2d(sim.angularVelocityRadPerSec)
    }
    override fun setVoltage(outputVolts: Double) {
        volts = outputVolts.coerceIn(-12.0, 12.0)
        sim.setInputVoltage(volts)
    }
}
