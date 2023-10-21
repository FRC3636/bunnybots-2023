package frc.robot.subsystems.targetvision


import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.commands.AimingConstants
import frc.robot.subsystems.drivetrain.Drivetrain
import frc.robot.utils.LimelightHelpers.LimelightTarget_Retro
import org.littletonrobotics.junction.Logger
import kotlin.math.tan

object TargetVision:  Subsystem {

    private val io = Limelight
    private val inputs: VisionIO.VisionIOInputs = VisionIO.VisionIOInputs()


    fun getDistance(target: LimelightTarget_Retro): Double{
        val angleToBucket = Units.degreesToRadians(LIMELIGHT_ANGLE + target.ty)
        return (BUCKET_HEIGHT - LIMELIGHT_HEIGHT) / tan(angleToBucket)
    }

    //ascending
    private val targetsbyDistance: List<Pair<LimelightTarget_Retro, Double>>
        get() {
            return inputs.targets.map {target: LimelightTarget_Retro -> Pair(target, getDistance(target))}.sortedBy {it.second}
        }
    val hasTargets: Boolean
        get(){
            return inputs.hasTargets
        }
    val curTime: Double
        get() {return inputs.lastTimeStampMS}


    override fun periodic() {
        Logger.getInstance().processInputs("Vision", inputs)


        if (hasTargets) {
            addMeasurement(takeMeasurement(closestTarget))
        }
        io.updateInputs(inputs)
    }

    data class Measurement(val timestamp: Double, val pose: Translation2d)

    fun takeMeasurement(target: LimelightTarget_Retro): Measurement{

        val curPose = Drivetrain.estimatedPose
        val targetTranslation = Translation2d(getDistance(target), target.tx)

        return Measurement(inputs.lastTimeStampMS, curPose.translation.plus(targetTranslation))

    }


    val closestTarget: LimelightTarget_Retro
        get(){
            return targetsbyDistance.map{pair: Pair<LimelightTarget_Retro, Double> -> pair.first}[0]
        }

    // samples
    var smaples: MutableList<Measurement> = mutableListOf()


    fun addMeasurement(measurement: Measurement){
        smaples.add(measurement)

        if (smaples.size > AimingConstants.SAMPLE_NUM)
            smaples.removeFirst()

    }

    fun reset(){
        smaples = mutableListOf()
    }




     private const val LIMELIGHT_ANGLE = 110 // degrees
     private const val LIMELIGHT_HEIGHT = 27.33 // inches
     private const val BUCKET_HEIGHT = 49 // inches. rough guess.//TODO unrough the guess


}