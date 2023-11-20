package frc.robot.commands

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.indexer.Indexer

class AutoIndex : Command {
    private val forceIndexTimeout = Timer()
    private var forcingIndex = false

    override fun execute() {
        if (forcingIndex) {
            Indexer.setSpeed(1.0)
            return
        }

        if (forceIndexTimeout.hasElapsed(7.0)) {
            forcingIndex = false
            forceIndexTimeout.stop()
            forceIndexTimeout.reset()
        }

        if(Indexer.beamUnbroken) {
            Indexer.setSpeed(0.0)
        } else {
            forcingIndex = true
            forceIndexTimeout.start()
        }
    }

    override fun getRequirements() = setOf(Indexer)

}
