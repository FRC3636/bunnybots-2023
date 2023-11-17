package frc.robot.subsystems.targetvision


import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.utils.LimelightHelpers.LimelightTarget_Retro
import org.littletonrobotics.junction.Logger
import kotlin.math.tan

object TargetVision : Subsystem {
    private val io: TargetVisionIO = Limelight
    private val inputs: TargetVisionIO.Inputs = TargetVisionIO.Inputs()
    private const val SAMPLE_NUM = 8

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.getInstance().processInputs("Target Vision", inputs)

        if (inputs.targets.isNotEmpty()) {
            addMeasurement(takeMeasurement(targetsByDistance[0].first))
        }
    }

    // in ascending order
    private val targetsByDistance: List<Pair<LimelightTarget_Retro, Double>>
        get() {
            return inputs.targets.map { target: LimelightTarget_Retro -> Pair(target, distanceToTarget(target)) }
                .sortedBy { it.second }
        }

    private var measurements: MutableList<Measurement> = mutableListOf()

    data class Measurement(val timestamp: Double, val translation: Translation2d)

    private fun takeMeasurement(target: LimelightTarget_Retro): Measurement {
        val targetTranslation = Translation2d(distanceToTarget(target), target.tx)

        return Measurement(timestamp = inputs.lastUpdateTimestamp, translation = targetTranslation)
    }

    private fun addMeasurement(measurement: Measurement) {
        measurements.add(measurement)

        if (measurements.size > SAMPLE_NUM) {
            measurements.removeFirst()
        }
    }

    private fun distanceToTarget(target: LimelightTarget_Retro): Double {
        val angleToBucket = Units.degreesToRadians(CAMERA_PITCH.degrees + target.ty)
        return (BUCKET_HEIGHT - LIMELIGHT_HEIGHT) / tan(angleToBucket)
    }

    // Constants
    private val CAMERA_PITCH = Rotation2d.fromDegrees(110.0)
    private val LIMELIGHT_HEIGHT = inchesToMeters(27.33)
    private val BUCKET_HEIGHT = inchesToMeters(49.0) // TODO: this is a rough guess, measure exactly
}
