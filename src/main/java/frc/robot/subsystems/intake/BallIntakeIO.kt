package frc.robot.subsystems.intake

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import frc.robot.CANDevice
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

public interface IntakeIO  {
}

    class IntakeInputs : LoggableInputs {
        var position = Rotation2d()
        var velocity = Rotation2d()
        var rollers = Rotation2d()

            override fun toLog(table: LogTable?) {
                table?.put("Yeet Main Motor Velocity", position.radians)
                table?.put("Yeet Secondary Motor Velocity", velocity.radians)


            }


            override fun fromLog(table: LogTable?) {
                table?.getDouble("Main Motor Velocity", velocity.radians)?.let { velocity = Rotation2d(it) }
                table?.getDouble("Secondary Motor Velocity", position.radians)?.let { position = Rotation2d(it) }
            }
    }

    fun updateInputs(inputs: IntakeInputs)


    fun setSpeed(speed: Double)


    fun setVoltage(outputVolts: Double) {}
}


class IntakeIOReal(PrimaryMotorCAN: CANDevice, SecondaryMotorCAN: CANDevice) : IntakeIO {



    private val motor = CANSparkMax(BallIntake, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    private val secondaryMotor = CANSparkMax(SecondaryMotorCAN.id, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val secondaryEncoder = secondaryMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)


    init {
        encoder.positionConversionFactor = Units.rotationsToRadians(1.0) *  GEAR_RATIO
        encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * GEAR_RATIO / 60
        motor.encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * GEAR_RATIO / 60
        motor.encoder.positionConversionFactor = Units.rotationsToRadians(1.0) * GEAR_RATIO / 60
    }

    override fun updateInputs(inputs: IntakeInputs){
        inputs.position = Rotation2d(encoder.position)
        inputs.velocity = Rotation2d(encoder.velocity)
        inputs.updateRaw()
        TODO("Not yet implemented")
    }
    override fun updateInputs(inputs: IntakeIO.IntakeRawInputs) {
    }

    override  fun setSpeed(speed: Double){
        motor.set(speed)
    }

    override fun setVoltage(outputVolts: Double){
        motor.setVoltage(outputVolts)
    }


    internal companion object Constants {
        const val GEAR_RATIO = 1.0


    }
}
