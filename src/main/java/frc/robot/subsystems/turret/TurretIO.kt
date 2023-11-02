package frc.robot.subsystems.turret

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import frc.robot.CANDevice
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface TurretIO  {

    class TurretRawInputs : LoggableInputs{

        var position: Double = 0.0
        var rotationalVelocity = 0.0


         override fun toLog(table: LogTable?){
            table?.put("Position", position)
             table?.put("Velocity", rotationalVelocity)
        }

        override fun fromLog(table: LogTable?){
            table?.getDouble("Position", position)?.let {position = it}
            table?.getDouble("Velocity", rotationalVelocity)?.let {rotationalVelocity = it}
        }

    }

     fun updateInputs(inputs: TurretInputs)

     fun setSpeed(speed: Double)

     fun setVoltage(outputVolts: Double) {}

}

class TurretInputs {
    private val inputs = TurretIO.TurretRawInputs()

    var position = Rotation2d()

    var velocity = Rotation2d()

    fun updateRaw() {
        inputs.position = position.radians
        inputs.rotationalVelocity = velocity.radians
    }

}

class TurretIOReal(motorCAN: CANDevice) : TurretIO {


    private val motor = CANSparkMax(motorCAN.id, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)


    init {
        encoder.positionConversionFactor = Units.rotationsToRadians(1.0) *  GEAR_RATIO
        encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * GEAR_RATIO / 60
        motor.encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * GEAR_RATIO / 60
        motor.encoder.positionConversionFactor = Units.rotationsToRadians(1.0) * GEAR_RATIO / 60
    }

    override fun updateInputs(inputs: TurretInputs){
        inputs.position = Rotation2d(encoder.position)
        inputs.velocity = Rotation2d(encoder.velocity)
        inputs.updateRaw()

    }

    override  fun setSpeed(speed: Double){
        motor.set(speed)
    }

    override fun setVoltage(outputVolts: Double){
        motor.setVoltage(outputVolts)
    }


    internal companion object Constants {
        //TODO find turret gear ratio :0
        const val GEAR_RATIO = 1.0

    }
}

class TurretIOSim : TurretIO {
    //TODO implement sim turret
    override fun updateInputs(inputs: TurretInputs) {

    }

    override fun setSpeed(speed: Double) {

    }

}