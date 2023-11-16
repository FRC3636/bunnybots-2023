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


    fun setArmSpeed(speed: Double)


    fun setArmVoltage(outputVolts: Double) {}

    fun setRollerSpeed(speed: Double)


    fun setRollerVoltage(outputVolts: Double) {}
}


class IntakeIOReal(ArmMotorID: Int, RollersMotorID: Int) : IntakeIO {



    private val armMotor = CANSparkMax(ArmMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val armEncoder = armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    private val rollerMotor = CANSparkMax(RollersMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val rollerEncoder = rollerMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)


    init {
        armEncoder.positionConversionFactor = Units.rotationsToRadians(1.0) *  GEAR_RATIO
        armEncoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * GEAR_RATIO / 60
        armMotor.encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * GEAR_RATIO / 60
        armMotor.encoder.positionConversionFactor = Units.rotationsToRadians(1.0) * GEAR_RATIO / 60
    }

    override fun updateInputs(inputs: IntakeIO.IntakeInputs){
        inputs.position = Rotation2d(armEncoder.position)
        inputs.velocity = Rotation2d(armEncoder.velocity)

        TODO("Not yet implemented")
    }

    override fun setArmSpeed(speed: Double){
        armMotor.set(speed)
    }

    override fun setArmVoltage(outputVolts: Double){
        armMotor.setVoltage(outputVolts)
    }


    override fun setRollerSpeed(speed: Double){
        rollerMotor.set(speed)
    }

    override fun setRollerVoltage(outputVolts: Double){
        rollerMotor.setVoltage(outputVolts)
    }


    internal companion object Constants {
        const val GEAR_RATIO = 1.0
    }
}
