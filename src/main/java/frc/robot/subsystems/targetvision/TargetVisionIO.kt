package frc.robot.subsystems.targetvision

import com.fasterxml.jackson.databind.ObjectMapper
import edu.wpi.first.networktables.NetworkTableEvent
import edu.wpi.first.networktables.NetworkTableEvent.Kind
import edu.wpi.first.networktables.NetworkTableInstance
import frc.robot.utils.LimelightHelpers
import frc.robot.utils.LimelightHelpers.LimelightResults
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import java.util.*


interface TargetVisionIO {

    class TargetVisionIOInputs : LoggableInputs {


        var hasTargets: Boolean = false
        var targetCount = 0
        var targets: List<LimelightHelpers.LimelightTarget_Retro> = mutableListOf()
        var latencyms: Double = 0.0
        var lastTimeStampMS: Double = 0.0


        override fun toLog(table: LogTable?) {
            table?.put("HasTargets", hasTargets)
            table?.put("TargetCount", targetCount.toDouble())
            table?.put("lastTimeStamp", lastTimeStampMS)
        }

        override fun fromLog(table: LogTable?) {
            table?.getBoolean("HasTargets", hasTargets)?.let { hasTargets = it }
            table?.getDouble("TargetCount", targetCount.toDouble())?.let { targetCount = it.toInt() }
            table?.getDouble("lastTimeStamp", lastTimeStampMS)?.let { lastTimeStampMS = it }

        }

    }
    fun updateInputs(inputs: TargetVisionIOInputs)

}

object Limelight : TargetVisionIO {
    private var hasTargets = false
    private var targets: List<LimelightHelpers.LimelightTarget_Retro> = mutableListOf()
    private var targetCount = 0
    private var lastTimeStamp: Double = 0.0

    private val table = NetworkTableInstance.getDefault().getTable("limelight")
    private val dumpSubscriber = table.getStringTopic("json").subscribe("no json :(")
    private val cl = table.getDoubleTopic("cl").subscribe( 0.0)
    private val tl = table.getDoubleTopic("tl").subscribe(0.0)
    private val ts = table.getStringTopic("ts").subscribe(":3")

    val curTimestamp: Double
        get() {
            val updates = ts.readQueue()
            val lastUpdate = updates[updates.size-1]
            val latency: Double = cl.get() + tl.get()
            return (lastUpdate.timestamp * 1e-6) * (latency * 1e-3)
        }

    override fun updateInputs(inputs: TargetVisionIO.TargetVisionIOInputs) {

        inputs.hasTargets = hasTargets
        inputs.targets = targets
        inputs.lastTimeStampMS = curTimestamp

    }


    init {
        val nt = NetworkTableInstance.getDefault()

        nt.addListener(
            dumpSubscriber,
            EnumSet.of(Kind.kValueAll),
            ::eventListener
        )

    }


    private fun eventListener(event: NetworkTableEvent) {
        val results = ObjectMapper().readValue(dumpSubscriber.get(), LimelightResults::class.java)
        hasTargets = results.targetingResults.targets_Retro.isNotEmpty()
        targets = results.targetingResults.targets_Retro.toList()
        targetCount = results.targetingResults.targets_Retro.size

    }
}

