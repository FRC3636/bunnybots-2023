package frc.robot.utils

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState

enum class DrivetrainCorner {
    FRONT_LEFT, FRONT_RIGHT, BACK_RIGHT, BACK_LEFT,
}

data class PerCorner<T>(
    val frontLeft: T, val frontRight: T, val backRight: T, val backLeft: T
) : Collection<T> {
    operator fun get(corner: DrivetrainCorner): T = when (corner) {
        DrivetrainCorner.FRONT_LEFT -> frontLeft
        DrivetrainCorner.FRONT_RIGHT -> frontRight
        DrivetrainCorner.BACK_RIGHT -> backRight
        DrivetrainCorner.BACK_LEFT -> backLeft
    }

    fun <U> map(block: (T) -> U): PerCorner<U> = generate { corner -> block(this[corner]) }
    fun <U> mapWithCorner(block: (T, DrivetrainCorner) -> U): PerCorner<U> =
        generate { corner -> block(this[corner], corner) }


    fun <U> zip(second: PerCorner<U>): PerCorner<Pair<T, U>> = generate { corner -> Pair(this[corner], second[corner]) }

    private fun sequence(): Sequence<T> = sequenceOf(frontLeft, frontRight, backRight, backLeft)

    override fun iterator(): Iterator<T> = sequence().iterator()

    override val size: Int = 4

    override fun isEmpty(): Boolean = false

    override fun containsAll(elements: Collection<T>): Boolean = elements.all { contains(it) }

    override fun contains(element: T): Boolean = sequence().contains(element)


    companion object {
        fun <T> generate(block: (DrivetrainCorner) -> T): PerCorner<T> = PerCorner(
            frontLeft = block(DrivetrainCorner.FRONT_LEFT),
            frontRight = block(DrivetrainCorner.FRONT_RIGHT),
            backRight = block(DrivetrainCorner.BACK_RIGHT),
            backLeft = block(DrivetrainCorner.BACK_LEFT)
        )

        internal fun <T> fromConventionalArray(array: Array<T>): PerCorner<T> =
            generate { corner -> array[corner.ordinal] }
    }
}

fun SwerveModuleState.toTranslation2dPerSecond(): Translation2d = Translation2d(this.speedMetersPerSecond, this.angle)

fun SwerveDriveKinematics.toCornerSwerveModuleStates(speeds: ChassisSpeeds): PerCorner<SwerveModuleState> =
    PerCorner.fromConventionalArray(toSwerveModuleStates(speeds))

fun SwerveDriveKinematics.cornerStatesToChassisSpeeds(states: PerCorner<SwerveModuleState>): ChassisSpeeds =
    toChassisSpeeds(*states.toList().toTypedArray())

fun SwerveDriveKinematics(translations: PerCorner<Translation2d>) =
    SwerveDriveKinematics(*translations.toList().toTypedArray())