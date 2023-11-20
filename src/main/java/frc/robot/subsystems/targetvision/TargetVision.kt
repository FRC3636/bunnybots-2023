package frc.robot.subsystems.targetvision


import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.utils.LimelightHelpers.LimelightTarget_Detector
import org.littletonrobotics.junction.Logger
import java.util.*
import kotlin.math.abs
import kotlin.math.tan
import kotlin.math.pow


object TargetVision:  Subsystem {

    private val io: TargetVisionIO = Limelight
    private val inputs: TargetVisionIO.Inputs = TargetVisionIO.Inputs()


    private fun getDistance(target: LimelightTarget_Detector): Double{
        val angleToBucket = Units.degreesToRadians(CAMERA_PITCH.degrees + target.ty)
        return (BUCKET_HEIGHT - LIMELIGHT_HEIGHT) / tan(angleToBucket)
    }

    //ascending
    private val targetsbyDistance: List<Pair<LimelightTarget_Detector, Double>>
        get() {
            return inputs.targets.map {target: LimelightTarget_Detector -> Pair(target, getDistance(target))}.sortedBy {it.second}
        }
    val hasTargets: Boolean
        get(){
            return inputs.targets.isNotEmpty()
        }
    val curTime: Double
        get() {return inputs.lastUpdateTimestamp}

    private val opposingAllianceId = if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        // opposing alliance is opposite of our current, so this is the id for red
        1.0
    } else {
        0.0
    }



    override fun periodic() {
        io.updateInputs(inputs)

        Logger.getInstance().processInputs("Vision", inputs)
        for (target in targetsbyDistance) {
            if(target.first.classID == opposingAllianceId){
                
                addMeasurement(takeTargetSample(target.first))
            }
        }


    }

   data class TargetSample(val timestamp: Double, val pose: LimelightTarget_Detector)

    data class PoseSample(val timestamp : Double, val pose: Translation2d)

    private fun takeTargetSample(target: LimelightTarget_Detector): TargetSample{

       //val targetTranslation = Translation2d(getDistance(target), target.tx)

       return TargetSample(inputs.lastUpdateTimestamp, target)

   }



    val primaryTargetSamples: List<PoseSample>
        get(){
            return robotSamples[0].map { pair -> PoseSample(pair.timestamp, Translation2d(getDistance(pair.pose), pair.pose.tx))}
        }


    val primaryTarget: LimelightTarget_Detector
        get(){
            return robotSamples[0].last().pose
        }


    private var robotSamples: MutableList<MutableList<TargetSample>> = mutableListOf()


    private fun groupConfidence(last: LimelightTarget_Detector, measurement: LimelightTarget_Detector): Double
    {
        var score = 0.0

        val deltaX = abs(measurement.tx - last.tx)
        val deltaY = abs(measurement.ty - last.ty)
        val deltaA = abs(measurement.ta.pow(0.5) - last.ta.pow(0.5))

        score += (TX_WEIGHT)*( 1 - (deltaX.pow(TX_DROPOFF_RATE) / 54.0.pow(TX_DROPOFF_RATE)) ) //27 = horizontal fov of limelight
        score += (TY_WEIGHT)*( 1 - (deltaY.pow(TY_DROPOFF_RATE) / 41.0.pow(TY_DROPOFF_RATE)) ) //20.5 = vertical fov of limelight
        score += (TA_WEIGHT)*( 1 - (deltaA.pow(TA_DROPOFF_RATE) / TA_MAX.pow(TA_DROPOFF_RATE)) )

        return score
    }


    private fun addMeasurement(measurement: TargetSample){


        // group targets
        for (samples in robotSamples) {
            // TODO: Find good threshold values for grouping
            if(groupConfidence(samples.last().pose, measurement.pose) > GROUPING_CONFIDENCE_THRESHOLD) {
                samples.add(measurement)
                if ((measurement.timestamp - samples.first().timestamp > MAX_SAMPLE_TIME_MS)) { // TODO: find good value
                    // cleanup old targets
                    samples.removeFirst()
                }
                return
            } else if((measurement.timestamp - samples.last().timestamp > MAX_SAMPLE_TIME_MS)){
                robotSamples.remove(samples)
            }
        }

        robotSamples.add(mutableListOf(measurement))
        Collections.swap(robotSamples, findBestRobot(robotSamples), 0)

   }


    //finds robot smaple collection that is easiest to hit and sets it as the primary target
    private fun findBestRobot(robotSamples: List<List<TargetSample>>): Int{

        fun getVelocity(samples: IndexedValue<List<TargetSample>>): Double{
            val tVector: List<Double> = samples.value.map{(timestamp, _) -> timestamp}
            val txVector: List<Double> = samples.value.map{(_, target) -> target.tx}

            // regressing tx
            return tVector.zip(txVector).sumOf { pair -> pair.first * pair.second } /
                    tVector.zip(tVector).sumOf { pair -> pair.first * pair.second }

        }


        var robotSamplesIndexed = robotSamples.withIndex()

        //filter by tx
        robotSamplesIndexed = robotSamplesIndexed.sortedBy { pair -> abs(pair.value.last().pose.tx)}
        val lowestTx = abs(robotSamplesIndexed[0].value.last().pose.tx)
        robotSamplesIndexed = robotSamplesIndexed.filter{ pair -> abs(pair.value.last().pose.tx) in lowestTx..lowestTx+TX_THRESHOLD}

        //filter by distance
        robotSamplesIndexed = robotSamplesIndexed.sortedBy {pair -> getDistance(pair.value.last().pose)}
        val lowestDistance = getDistance(robotSamplesIndexed[0].value.last().pose)
        robotSamplesIndexed = robotSamplesIndexed.filter{ pair -> getDistance(pair.value.last().pose) in lowestDistance..lowestDistance + DISTANCE_THRESHOLD}


        return robotSamplesIndexed.minByOrNull { getVelocity(it) }!!.index


    }


     private val CAMERA_PITCH = Rotation2d.fromDegrees(110.0) // degrees
     private const val LIMELIGHT_HEIGHT = 27.33 // inches
     private const val BUCKET_HEIGHT = 49 // inches. rough guess. TODO unrough the guess

    private const val MAX_SAMPLE_TIME_MS = 2000

    //grouping
    private const val GROUPING_CONFIDENCE_THRESHOLD = 0.80
    private const val TX_WEIGHT = 0.75
    private const val TX_DROPOFF_RATE = 0.25
    private const val TA_WEIGHT = 0.15
    private const val TA_DROPOFF_RATE = 1
    private const val TA_MAX = 30.0 // verry rough guess, need to find how fast ta increases decreases as target moves away
    private const val TY_WEIGHT = 0.1
    private const val TY_DROPOFF_RATE = 1.2

    //preferences

    private const val TX_THRESHOLD = 5.0
    private const val DISTANCE_THRESHOLD = 35.0 //inches




}