package frc.robot.commands

import com.github.chen0040.glm.solvers.Coefficients
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.targetvision.TargetVision
import frc.robot.subsystems.targetvision.TargetVision.Sample
import frc.robot.subsystems.turret.Turret
import frc.robot.utils.QuadraticPolynomial
import org.ejml.simple.SimpleMatrix
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt

class ControlWithJoystick(private val joystickX: DoubleSupplier, private val joystickY: DoubleSupplier ) : CommandBase(){

    override fun execute() {
        val angle = atan2(joystickY.asDouble, joystickX.asDouble)
        Turret.setTarget(Rotation2d(angle))
    }

}

class TrackPrimary() : CommandBase(){
    init {
        addRequirements(Turret)
    }

    override fun execute() {
        if(TargetVision.hasTargets) {

            Turret.setTarget(Turret.angleToChassis.plus(Rotation2d.fromDegrees(TargetVision.primaryTarget.tx)))
        }

    }
}

//threshold to stop newton raphson method, in milliseconds
internal const val THRESHOLD = 0.001
internal const val INITIAL_SEED = 4.0



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

        var h = func(nextSeed)/funcDerivative(nextSeed)

        while(abs(h) >= THRESHOLD){

            h = func(nextSeed)/funcDerivative(nextSeed)
            nextSeed -= h

        }
        return nextSeed
    }



    private fun findTargetRotation(): Rotation2d {
        //samples
        //offset timestamps so 0 is the current time
        val smaples =
            setSampleEpoch(TargetVision.primaryTargetSamples as MutableList<Sample>, TargetVision.curTime)

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


        val x = QuadraticPolynomial(B.get(2, 0), B.get(1, 0), B.get(0, 0))
        val y =QuadraticPolynomial(B.get(2, 1), B.get(1, 1), B.get(0, 1))

        val vx = x.getDerivative()
        val vy = y.getDerivative()
        val tofDerivative = timeOfFlight.getDerivative()

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
            Turret.setTarget(getRotationalOffset())
        }
    }


}



fun setSampleEpoch(smaples: MutableList<Sample>, epoch: Double): List<Sample>
    = smaples.map { Sample(it.timestamp - epoch, it.pose)}
