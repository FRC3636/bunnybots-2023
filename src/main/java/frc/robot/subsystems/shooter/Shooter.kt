package frc.robot.subsystems.shooter

import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.CANDevice
import frc.robot.utils.LimelightHelpers
import kotlin.math.tan

object Shooter : Subsystem {

    // TODO: Implement feedforward maybe

    private const val LIMELIGHT_ANGLE = 110 // degrees
    private const val LIMELIGHT_HEIGHT = 27.33 // inches
    private const val BUCKET_HEIGHT = 49 // inches. rough guess.

    private val shooterInputs = ShooterInputs()

    val io = ShooterIOReal(CANDevice.ShooterMotorMain, CANDevice.ShooterMotorSecondary)

    override fun periodic() {
        val latestResults = LimelightHelpers.getLatestResults("limelight")
        if (latestResults.targetingResults.targets_Retro.isNotEmpty()) {
            val ty = LimelightHelpers.getTY("limelight")
            val angleToBucket = Units.degreesToRadians(LIMELIGHT_ANGLE + ty)
            val distanceToBucket = (BUCKET_HEIGHT - LIMELIGHT_HEIGHT) / tan(angleToBucket)
            println("Distance to bucket is $distanceToBucket inches!")
        } else {
            println("No stuff to measure :((")
        }
    }

    fun shoot() {
        throw NotImplementedError("Not implemented")
    }
}