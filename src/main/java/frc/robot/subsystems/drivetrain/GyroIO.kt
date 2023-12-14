package frc.robot.subsystems.drivetrain

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.geometry.Quaternion
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs




interface GyroIO {

    class GyroRawInputs : LoggableInputs{

          var rotationYaw: Double = 0.0
          var rotationPitch: Double = 0.0
          var rotationRoll: Double = 0.0

          override fun toLog(table: LogTable?){
              table?.put("RotationYaw", rotationYaw)
              table?.put("RotationPitch", rotationPitch)
              table?.put("RotationRoll", rotationRoll)
          }

          override fun fromLog(table: LogTable?){
              table?.getDouble("RotationYaw", rotationYaw)?.let { rotationYaw = it }
              table?.getDouble("RotationPitch", rotationPitch)?.let {rotationPitch = it}
              table?.getDouble("RotationRoll", rotationRoll)?.let {rotationRoll = it}

          }
     }

    fun updateInputs(inputs: GyroInputs)

    fun calibrate() {}

    fun setRotation(rotation: Rotation3d) {}
}

class GyroInputs {
    private val rawInputs = GyroIO.GyroRawInputs()
    var rotation: Rotation3d = Rotation3d(0.0,0.0,0.0)

     fun updateRaw(){
        rawInputs.rotationRoll = rotation.x
        rawInputs.rotationPitch = rotation.y
        rawInputs.rotationYaw = rotation.z
    }

}




class NavXGyroIO(private var offset: Rotation3d = Rotation3d()) : GyroIO{

    private val ahrs = AHRS()
    private val rotationOffset = Rotation2d()

      override fun updateInputs(inputs: GyroInputs){

          inputs.rotation = offset.rotateBy(
              Rotation3d(
                  Quaternion(
                      ahrs.quaternionW.toDouble(),
                      ahrs.quaternionX.toDouble(),
                      ahrs.quaternionY.toDouble(),
                      ahrs.quaternionZ.toDouble()
                  )
              )
          )
          inputs.updateRaw()
    }

    override fun setRotation(rotation: Rotation3d){
        ahrs.reset()
        offset = rotation
    }

    override fun calibrate() {
        ahrs.calibrate()
    }

}

class SimGyroIO : GyroIO{

    override fun updateInputs(inputs: GyroInputs){
        inputs.rotation = Rotation3d()
        inputs.updateRaw()
    }
}