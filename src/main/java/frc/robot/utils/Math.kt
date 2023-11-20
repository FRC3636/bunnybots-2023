package frc.robot.utils

import kotlin.math.PI
import kotlin.math.pow

const val TAU: Double = PI * 2

class QuadraticPolynomial(var a: Double, var b: Double, var c: Double){
    fun of(x: Double): Double {
        return (a * x).pow(2) + (b * x) + c
    }

    val derivative: QuadraticPlynomial
        get() {
            return QuadraticPolynomial(0.0, (2 * a), b)
        }

}