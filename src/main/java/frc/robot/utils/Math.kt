package frc.robot.utils

import kotlin.math.PI

const val TAU: Double = PI * 2

infix fun <A,B,C,D> ((A,B) -> C).compose(other: (C) -> D): (A, B) -> D{
    return { A,B -> other(this(A,B)) }
}
