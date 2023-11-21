package frc.robot.commands


import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.targetvision.TargetVision
import frc.robot.subsystems.targetvision.TargetVision.PoseSample
import frc.robot.subsystems.turret.Turret
import frc.robot.utils.QuadraticPolynomial
import org.ejml.simple.SimpleMatrix
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt

//threshold to stop newton raphson method, in milliseconds
internal const val THRESHOLD = 0.001
internal const val INITIAL_SEED = 4.0
internal const val MAX_LOOPS = 100



class AimAtTarget() : CommandBase(){

    init {
        addRequirements(Turret)
    }



    private val timeOfFlight = QuadraticPolynomial(0.0, 0.0, 0.0)


    private val euclideanNorm: (x: Double, y: Double) -> Double =
        {x,y -> sqrt(x.pow(2) + y.pow(2))}

    private fun euclideanNormDerivative(vx: QuadraticPolynomial, vy: QuadraticPolynomial, x: QuadraticPolynomial, y: QuadraticPolynomial): (Double) -> Double
    {
        return {t: Double  -> ( (vx.of(t) + vy.of(t)) / (2*sqrt( x.of(t).pow(2) + y.of(t).pow(2) ) ) ) }
    }



    //iteratively find roots of function by offsetting a seed based on x intercept of tangent line of last guess
    //TODO account for chaotic fractal thingie
    private fun newtonRaphson(func: (Double) -> Double, funcDerivative: (Double) -> Double ): Double{

        val seed = INITIAL_SEED
        var nextSeed = seed

        var h: Double

        for( i in 1..MAX_LOOPS){
            h = func(nextSeed)/funcDerivative(nextSeed)
            nextSeed -= h
            if(abs(h) >= THRESHOLD) { break }

        }
        return nextSeed
    }



    private fun findTargetRotation(): Rotation2d {
        //samples
        //offset timestamps so 0 is the current time
        val smaples =
            setSampleEpoch(TargetVision.primaryTargetSamples as MutableList<PoseSample>, TargetVision.curTime)

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
        // Read more: https://en.wikipedia.org/wiki/Moore–Penrose_inverse
        val XtX = designMatrix.transpose().mult(designMatrix)
        val XtXInv = XtX.invert()
        val XtY = designMatrix.transpose().mult(responseMatrix)
        val B = XtXInv.mult(XtY)


        val x = QuadraticPolynomial(B.get(2, 0), B.get(1, 0), B.get(0, 0))
        val y =QuadraticPolynomial(B.get(2, 1), B.get(1, 1), B.get(0, 1))

        val vx = x.derivative
        val vy = y.derivative
        val tofDerivative = timeOfFlight.derivative

        //TODO add link to docs explaining what this means
        //TODO create docs to explain what this means
        val solveFunction: (t: Double) -> Double = { t -> timeOfFlight.of(euclideanNorm(x.of(t),y.of(t)))}
        val solveDerivative: (t: Double) -> Double = { t -> (tofDerivative.of(euclideanNorm(x.of(t),y.of(t))))*euclideanNormDerivative(vx, vy, x, y)(t)}

        val optimalTimeOffset = newtonRaphson(solveFunction, solveDerivative)

        val predictedTranslation = Translation2d(x.of(optimalTimeOffset), y.of(optimalTimeOffset))

        val shotPoint = TargetVision.primaryTargetSamples[TargetVision.primaryTargetSamples.size - 1].pose.plus(predictedTranslation)
        return Rotation2d(atan2(shotPoint.y, shotPoint.x))
    }


    override fun execute() {
        if (TargetVision.hasTargets) {
            Turret.setTarget(findTargetRotation())
        }
    }


}



fun setSampleEpoch(smaples: MutableList<PoseSample>, epoch: Double): List<PoseSample>
    = smaples.map { PoseSample(it.timestamp - epoch, it.pose)}
