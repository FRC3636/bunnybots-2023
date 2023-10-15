package frc.robot.utils

import com.revrobotics.SparkMaxPIDController
import edu.wpi.first.math.controller.PIDController

data class PIDCoefficients(
    val p: Double = 0.0,
    val i: Double = 0.0,
    val d: Double = 0.0
)

fun PIDController(cs: PIDCoefficients) =
    PIDController(cs.p, cs.i, cs.d)

var PIDController.constants: PIDCoefficients
    get() = PIDCoefficients(
        p = p,
        i = i,
        d = d
    )
    set(constants) {
        p = constants.p
        i = constants.i
        d = constants.d
    }

var SparkMaxPIDController.constants: PIDCoefficients
    get() = PIDCoefficients(
        p = p,
        i = i,
        d = d,
    )
    set(constants) {
        p = constants.p
        i = constants.i
        d = constants.d
    }