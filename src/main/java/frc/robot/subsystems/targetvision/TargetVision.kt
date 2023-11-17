package frc.robot.subsystems.targetvision


import edu.wpi.first.math.Num
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.utils.LimelightHelpers.LimelightTarget_Detector
import org.littletonrobotics.junction.Logger
import kotlin.math.abs
import kotlin.math.tan
import kotlin.math.pow
import edu.wpi.first.math.Vector


object TargetVision:  Subsystem {

    private val io: TargetVisionIO = Limelight
    private val inputs: TargetVisionIO.TargetVisionIOInputs = TargetVisionIO.TargetVisionIOInputs()
    private const val SAMPLE_NUM = 8


    private fun getDistance(target: LimelightTarget_Detector): Double{
        val angleToBucket = Units.degreesToRadians(CAMERA_PITCH + target.ty)
        return (BUCKET_HEIGHT - LIMELIGHT_HEIGHT) / tan(angleToBucket)
    }

    //ascending
    private val targetsbyDistance: List<Pair<LimelightTarget_Detector, Double>>
        get() {
            return inputs.targets.map {target: LimelightTarget_Detector -> Pair(target, getDistance(target))}.sortedBy {it.second}
        }
    val hasTargets: Boolean
        get(){
            return inputs.hasTargets
        }
    val curTime: Double
        get() {return inputs.lastTimeStampMS}



    override fun periodic() {
        Logger.getInstance().processInputs("Vision", inputs)

        for (target in targetsbyDistance) {
            if(target.first.classID == (OpposingAlliance.id)){
                
                addMeasurement(takeMeasurement(target.first))
            }
        }

        io.updateInputs(inputs)
    }

   data class Measurement(val timestamp: Double, val pose: LimelightTarget_Detector)

    data class Sample(val timestamp : Double, val pose: Translation2d)

    private fun takeMeasurement(target: LimelightTarget_Detector): Measurement{

       //val targetTranslation = Translation2d(getDistance(target), target.tx)

       return Measurement(inputs.lastTimeStampMS, target)

   }



    val primaryTargetSamples: List<Sample>
        get(){
            return robotPoses[0].map { pair -> Sample(pair.timestamp, Translation2d(getDistance(pair.pose), pair.pose.tx))}
        }


    val primaryTarget: LimelightTarget_Detector
        get(){
            return robotPoses[0].last().pose
        }


    var robotPoses: MutableList<MutableList<Measurement>> = mutableListOf()


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


    fun addMeasurement(measurement: Measurement){

        var grouped = false

        // group targets
        for (pose in robotPoses) {
            // TODO: Find good threshold values for grouping
            if(groupConfidence(pose.last().pose, measurement.pose) > GROUPING_CONFIDENCE_THRESHOLD) {
                pose.add(measurement)
                if ((measurement.timestamp - pose.first().timestamp > MAX_SAMPLE_TIME_MS)) { // TODO: find good value
                    // cleanup old targets
                    pose.removeFirst()
                }
                grouped = true
                break
            } else if((measurement.timestamp - pose.last().timestamp > MAX_SAMPLE_TIME_MS)){
                robotPoses.remove(pose)
            }
        }

        if(!grouped){
            robotPoses.add(mutableListOf(measurement))
        }





   }


    //give a measurement a rating based on how likely it is to be our current primary target
    private fun rateTrackingPreference(measurement: List<List<Measurement>>): Int{

        fun getVelocity(samples: Pair<List<Measurement>, Int>): Double{
            val tVector: List<Double> = samples.first.map{(timestamp, _) -> timestamp}
            val txVector: List<Double> = samples.first.map{(_, target) -> target.tx}

            // regressing tx
            return tVector.zip(txVector).sumOf { pair -> pair.first * pair.second } /
                    tVector.zip(tVector).sumOf { pair -> pair.first * pair.second }

        }

        var measurementIndexed = measurement.map { list -> Pair(list, measurement.indexOf(list))}

        //filter by tx
        measurementIndexed = measurementIndexed.sortedBy { pair -> abs(pair.first.last().pose.tx)}
        val lowestTx = abs(measurementIndexed[0].first.last().pose.tx)
        measurementIndexed = measurementIndexed.filter{ pair -> abs(pair.first.last().pose.tx) in lowestTx..lowestTx+TX_THRESHOLD}

        //filter by distance
        measurementIndexed = measurementIndexed.sortedBy {pair -> getDistance(pair.first.last().pose)}
        val lowestDistance = getDistance(measurementIndexed[0].first.last().pose)
        measurementIndexed = measurementIndexed.filter{ pair -> getDistance(pair.first.last().pose) in lowestDistance..lowestDistance + DISTANCE_THRESHOLD}


        return measurementIndexed.minByOrNull { getVelocity(it) }!!.second


    }


    private fun isBlueAlliance(): Boolean {
        return DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)
    }

    private val OpposingAlliance = if(isBlueAlliance()){Alliance.BLUE} else {Alliance.RED}

     enum class Alliance(val id: Double){
        RED(1.0),
        BLUE(0.0)
    }


     private const val CAMERA_PITCH = 110 // degrees
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
    private const val ANGLE_WEIGHT = 0.70
    private const val ANGULAR_VELOCITY_WEIGHT = 0.10
    private const val DISTANCE_WEIGHT = 0.20

    private const val TX_THRESHOLD = 5.0
    private const val DISTANCE_THRESHOLD = 35.0 //inches




}