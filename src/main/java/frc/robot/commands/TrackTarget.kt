package frc.robot.commands

import com.github.chen0040.data.frame.DataFrame
import com.github.chen0040.data.frame.DataQuery
import com.github.chen0040.data.frame.Sampler
import com.github.chen0040.glm.enums.GlmSolverType
import com.github.chen0040.glm.solvers.Coefficients
import com.github.chen0040.glm.solvers.Glm
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Turret.Turret
import frc.robot.subsystems.targetvision.TargetVision
import frc.robot.subsystems.targetvision.TargetVision.Measurement
import frc.robot.utils.LimelightHelpers.LimelightTarget_Retro
import frc.robot.utils.compose
import kotlin.math.abs
import kotlin.math.pow

import kotlin.math.sqrt


class TrackClosest(private val turret: Turret) : CommandBase(){


    var currentTarget: LimelightTarget_Retro = TargetVision.closestTarget

    private var mode: AimMode = AimMode.SEARCH



    init{
        addRequirements(turret)
    }

    override fun execute(){
        if(!TargetVision.hasTargets){ mode = AimMode.SEARCH}

        if(mode == AimMode.TRACK){
            Turret.setTarget(Turret.relativeAngle.plus(Rotation2d.fromDegrees(TargetVision.closestTarget.tx)))
        }else if(mode == AimMode.SEARCH){
            TargetVision.reset()

        }

        //TODO figure out if timestamping is fucked

    }





     enum class AimMode{
        TRACK,
        SEARCH,
    }

}

object AimingConstants{
    const val SAMPLE_NUM= 8
    const val EPSILON = 0.0001
    const val INITIAL_SEED = 4.0
    const val LEAD_TIME_MS = 300
}

class AimAtTarget(private val turret: Turret) : CommandBase(){

    var expectedTargetPose: Translation2d? = null

    private val schema: DataQuery.DataFrameQueryBuilder =
        DataQuery.blank()
            .newInput("t")
            .newInput("t^2")
            .newOutput("x/y")
            .end()

    private val glm: Glm = Glm.linear()

    private val targetPose: Pose2d? = null

    private enum class CoefficientsList(val coefficients: Coefficients){
        X(Coefficients()),
        Y(Coefficients()),
        Tof(Coefficients()),
    }


    private fun quadraticBuilder(coefficients: Coefficients): (Double) -> Double{
        val terms = coefficients.values
        return {input:Double -> input*terms[2].pow(2) +
                input*terms[1] + terms[0]
        }
    }

    private fun quadraticDerivativeBuilder(coefficients: Coefficients): (Double) -> Double{
        val terms = coefficients.values
        return { input:Double -> 2*input*terms[2] + input*terms[1] }
    }

    //TODO test for ToF Coefficients
    private val timeOfFlight: (distance: Double) -> Double = quadraticBuilder(CoefficientsList.Tof.coefficients)


    private var optimalTimeOffset: Double = 0.0

    private val pointsToDistance: (x: Double, y: Double) -> Double =
        {x,y -> sqrt(x.pow(2) + y.pow(2))}

    private val pointsToDistanceDerivative: (vx: (Double) -> Double, vy: (Double) -> Double, x: (Double) -> Double, y: (Double) -> Double) -> (Double) -> Double =
        { vx, vy, x, y ->  { t: Double  -> ( (vx(t) + vy(t)) / (2*sqrt( x(t).pow(2) + y(t).pow(2) ) ) )} }





    init{


        addRequirements(turret)

    }

    private fun newtonRaphson(seed: Double, func: (Double) -> Double, funcPrime: (Double) -> Double ): Double{

        var nextSeed = seed
        var h = func(nextSeed)/funcPrime(nextSeed)

        while(abs(h) >= AimingConstants.EPSILON){

            h = func(nextSeed)/funcPrime(nextSeed)
            nextSeed -= h

        }
        return nextSeed
    }
    override fun initialize() {
        val smaples = setSampleEpoch(TargetVision.smaples, TargetVision.curTime)

        val samplerX: Sampler.DataSampleBuilder = Sampler()
            .forColumn("t").generate{_, index -> smaples[index].timestamp}
            .forColumn("t^2").generate{_, index -> smaples[index].timestamp.pow(2)}
            .forColumn("x/y").generate {_, index -> smaples[index].pose.x}
            .end()

        val samplerY: Sampler.DataSampleBuilder = Sampler()
            .forColumn("t").generate{_, index -> smaples[index].timestamp}
            .forColumn("t^2").generate{_, index -> smaples[index].timestamp.pow(2)}
            .forColumn("x/y").generate {_, index -> smaples[index].pose.y}

            .end()

        var frameX: DataFrame = schema.build()
        var frameY: DataFrame = schema.build()

        frameX = samplerX.sample(frameX, AimingConstants.SAMPLE_NUM)
        frameY = samplerY.sample(frameY, AimingConstants.SAMPLE_NUM)


        glm.solverType = GlmSolverType.GlmIrls

        glm.fit(frameX)
        CoefficientsList.X.coefficients.values = glm.coefficients.values

        glm.fit(frameY)
        CoefficientsList.Y.coefficients.values = glm.coefficients.values


        val x = quadraticBuilder(CoefficientsList.X.coefficients)
        val y = quadraticBuilder(CoefficientsList.Y.coefficients)
        val vx = quadraticDerivativeBuilder(CoefficientsList.X.coefficients)
        val vy = quadraticDerivativeBuilder(CoefficientsList.Y.coefficients)
        val tofDerivative = quadraticDerivativeBuilder(CoefficientsList.Tof.coefficients)

        val lhsfunction: (t: Double) -> Double = { t -> pointsToDistance.compose(timeOfFlight)(x(t), y(t))}
        val lhsDerivative: (t: Double) -> Double = { t -> pointsToDistance.compose(tofDerivative)(x(t),y(t))*pointsToDistanceDerivative(vx, vy, x, y)(t)}

        optimalTimeOffset = newtonRaphson(AimingConstants.INITIAL_SEED, lhsfunction, lhsDerivative)
    }


    private val velocity: Double
        get() {
            return 0.0
        }





}

fun setSampleEpoch(smaples: MutableList<Measurement>, epoch: Double): List<Measurement>
    = smaples.map { it -> Measurement(it.timestamp - epoch, it.pose)}






