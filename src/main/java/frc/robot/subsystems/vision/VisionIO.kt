package frc.robot.subsystems.vision

import com.fasterxml.jackson.databind.ObjectMapper
import edu.wpi.first.networktables.NetworkTableEvent
import edu.wpi.first.networktables.NetworkTableEvent.Kind
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StringTopic
import frc.robot.utils.LimelightHelpers
import frc.robot.utils.LimelightHelpers.LimelightResults
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import java.util.*


interface VisionIO {

    class VisionIOInputs : LoggableInputs {


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
            table?.getBoolean("HasTargets", hasTargets)?.let { hasTargets = it}
            table?.getDouble("TargetCount", targetCount.toDouble())?.let { targetCount = it.toInt()}
            table?.getDouble("lastTimeStamp", lastTimeStampMS)?.let { lastTimeStampMS = it}

        }

    }


    fun updateInputs(inputs: VisionIOInputs)

}

object Limelight : VisionIO {
    private var hasTargets = false
    private var targets: List<LimelightHelpers.LimelightTarget_Retro> = mutableListOf()
    private var targetCount = 0
    private var lastTimeStamp: Double = 0.0

    private val table = NetworkTableInstance.getDefault().getTable("limelight")
    private val dumpSubscriber =table.getStringTopic("json").subscribe(null)
    private val limelightJson = LimelightHelpers.getLimelightNTTableEntry("limelight","json").topic;

    override fun updateInputs(inputs: VisionIO.VisionIOInputs) {

        inputs.hasTargets = hasTargets
        inputs.targets = targets
        inputs.lastTimeStampMS = lastTimeStamp
    }

   @Serializable
   data class limelightDock( val botpose_wpiblue: String, @SerialName("Retro") val targets: String)asdfghgfdsasdfghjhgfdsdfghjhgfd

    init {
        val nt = NetworkTableInstance.getDefault()

        StringTopic(LimelightHelpers.getLimelightNTTableEntry("limelight","json").topic).subscribe("no jsson :(")
        nt.addListener(
            LimelightHelpers.getLimelightNTTableEntry("limelight","json"),
            EnumSet.of(Kind.kValueAll),
            ::eventListener
        )

    }

    private fun eventListener(event: NetworkTableEvent) {
        val results = ObjectMapper().readValue(event.valueData.value.string, LimelightResults::class.java)
        hasTargets = results.targetingResults.targets_Retro.isNotEmpty()
        targets = results.targetingResults.targets_Retro.toList()
        targetCount = results.targetingResults.targets_Retro.size

    }




}

