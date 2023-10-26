package frc.robot.commands

import com.github.chen0040.glm.solvers.Coefficients
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Turret.Turret
import frc.robot.subsystems.targetvision.TargetVision
import frc.robot.subsystems.targetvision.TargetVision.Measurement
import org.ejml.simple.SimpleMatrix
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt


class TrackClosest() : CommandBase(){

    private var mode: AimMode = AimMode.SEARCH



    init {
        addRequirements(Turret)
    }

    override fun execute() {
        if(!TargetVision.hasTargets){ mode = AimMode.SEARCH}

        if(mode == AimMode.TRACK) {
            Turret.setTarget(Turret.relativeAngle.plus(Rotation2d.fromDegrees(TargetVision.closestTarget.tx)))
        }else if(mode == AimMode.SEARCH) {
            TargetVision.reset()

        }


    }

     enum class AimMode {
        TRACK,
        SEARCH,
    }

}

//threshold to stop newton raphson method, in milliseconds
internal const val THRESHOLD = 0.0001
internal const val INITIAL_SEED = 4.0



class AimAtTarget() : CommandBase(){

    init {
        addRequirements(Turret)
    }


    private val XCoefficients = Coefficients()
    private val YCoefficients = Coefficients()
    private val TofCoefficients = Coefficients()



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
    private val timeOfFlight: (distance: Double) -> Double = quadraticBuilder(TofCoefficients)


    private var optimalTimeOffset: Double = 0.0

    private val euclideanNorm: (x: Double, y: Double) -> Double =
        {x,y -> sqrt(x.pow(2) + y.pow(2))}

    private fun euclideanNormDerivative(vx: (Double) -> Double, vy: (Double) -> Double, x: (Double) -> Double, y: (Double) -> Double): (Double) -> Double{
        return {t: Double  -> ( (vx(t) + vy(t)) / (2*sqrt( x(t).pow(2) + y(t).pow(2) ) ) ) }
    }



    //iteratively find roots of function by offsetting a seed based on x intercept of tangent line of last guess
    //TODO account for chaotic fractal thingie
    private fun newtonRaphson(seed: Double, func: (Double) -> Double, funcDerivative: (Double) -> Double ): Double{

        var nextSeed = seed
        var h = func(nextSeed)/funcDerivative(nextSeed)

        while(abs(h) >= THRESHOLD){

            h = func(nextSeed)/funcDerivative(nextSeed)
            nextSeed -= h

        }
        return nextSeed
    }

    private fun getRotationalOffset(): Rotation2d{
        //samples

        //offset timestamps so 0 is the current time
        val smaples = setSampleEpoch(TargetVision.smaples, TargetVision.curTime)

        //Generalized Linear Model regresses points to find quadratic functions for translation x and y in terms of t
        // Construct the design matrix X with columns for 1 (intercept), t, and t^2
        val designMatrix = SimpleMatrix(smaples.size, 3)
        val responseMatrix = SimpleMatrix(smaples.size, 2)
        for (i in smaples.indices) {
            designMatrix[i, 0] = 1.0
            designMatrix[i, 1] = smaples[i].timestamp
            designMatrix[i, 2] = smaples[i].timestamp.pow(2)

            responseMatrix[i, 0] = smaples[i].pose.x
            responseMatrix[i, 1] = smaples[i].pose.y
        }

        // Solve for the coefficient matrix B using the formula B = (X'X)^-1 X'Y
        val XtX = designMatrix.transpose().mult(designMatrix)
        val XtXInv = XtX.invert()
        val XtY = designMatrix.transpose().mult(responseMatrix)
        val B = XtXInv.mult(XtY)

        XCoefficients.values = listOf(B.get(0,0),B.get(1,0),B.get(1,0))
        YCoefficients.values = listOf(B.get(0,1),B.get(1,1),B.get(2,1))

        val x = quadraticBuilder(XCoefficients)
        val y = quadraticBuilder(YCoefficients)
        val vx = quadraticDerivativeBuilder(XCoefficients)
        val vy = quadraticDerivativeBuilder(YCoefficients)
        val tofDerivative = quadraticDerivativeBuilder(TofCoefficients)

        //TODO add link to docs explaining what this means
        //TODO create docs to explain what this means
        val solveFunction: (t: Double) -> Double = { t -> timeOfFlight(euclideanNorm(x(t),y(t)))}
        val solveDerivative: (t: Double) -> Double = { t -> (tofDerivative(euclideanNorm(x(t),y(t))))*euclideanNormDerivative(vx, vy, x, y)(t)}



        optimalTimeOffset = newtonRaphson(INITIAL_SEED, solveFunction, solveDerivative)

        val predictedTranslation = Translation2d(x(optimalTimeOffset), y(optimalTimeOffset))

        val shotPoint = TargetVision.smaples[TargetVision.smaples.size - 1].pose.plus(predictedTranslation)
        return Rotation2d(atan2(shotPoint.x, shotPoint.y))
    }



    override fun execute() {
        if (TargetVision.hasTargets) {
            Turret.setTarget(getRotationalOffset())
        }
    }


}

fun setSampleEpoch(smaples: MutableList<Measurement>, epoch: Double): List<Measurement>
    = smaples.map { Measurement(it.timestamp - epoch, it.pose)}





