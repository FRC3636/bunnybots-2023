package frc.robot.subsystems.targetvision

import com.fasterxml.jackson.databind.ObjectMapper
import edu.wpi.first.networktables.NetworkTableEvent
import edu.wpi.first.networktables.NetworkTableEvent.Kind
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import frc.robot.utils.LimelightHelpers
import frc.robot.utils.LimelightHelpers.LimelightResults
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import java.util.*


interface TargetVisionIO {

    class Inputs : LoggableInputs {
        var currentTimestamp: Double = 0.0
        var lastUpdateTimestamp: Double = 0.0

        // FIXME: this isn't being logged, our turret will be useless in replay
        var targets: List<LimelightHelpers.LimelightTarget_Retro> = mutableListOf()


        override fun toLog(table: LogTable?) {
            table?.put("Current Timestamp", currentTimestamp)
            table?.put("Last Update Timestamp", lastUpdateTimestamp)
        }

        override fun fromLog(table: LogTable?) {
            table?.getDouble("Current Timestamp", 0.0)?.let { currentTimestamp = it }
            table?.getDouble("Last Update Timestamp", 0.0)?.let { lastUpdateTimestamp = it }
        }
    }

    fun updateInputs(inputs: Inputs)
}

object Limelight : TargetVisionIO {
    private val nt = NetworkTableInstance.getDefault()

    private var targets: List<LimelightHelpers.LimelightTarget_Retro> = emptyList()
    private var lastUpdateTimestamp = 0.0

    private val table = NetworkTableInstance.getDefault().getTable("limelight")
    private val dumpSubscriber = table.getStringTopic("json").subscribe("no json :(")

    override fun updateInputs(inputs: TargetVisionIO.Inputs) {
        inputs.currentTimestamp = Timer.getFPGATimestamp() + nt.serverTimeOffset.orElse(0)
        inputs.lastUpdateTimestamp = lastUpdateTimestamp
        inputs.targets = targets
    }


    init {
        nt.addListener(
            dumpSubscriber, EnumSet.of(Kind.kValueAll), ::eventListener
        )
    }


    private fun eventListener(event: NetworkTableEvent) {
        val results = ObjectMapper().readValue(dumpSubscriber.get(), LimelightResults::class.java)
        lastUpdateTimestamp = results.targetingResults.timestamp_RIOFPGA_capture
        targets = results.targetingResults.targets_Retro.toList()
    }
}

