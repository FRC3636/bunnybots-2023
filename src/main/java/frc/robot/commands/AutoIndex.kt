package frc.robot.commands

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.indexer.Indexer

class AutoIndex : Command {
    private val timer = Timer()
    private var didTimerStart = false

    override fun execute() {
        if(!Indexer.beamBreak) {
            if (!didTimerStart) {
                timer.start()
                didTimerStart = true
            }
            Indexer.setSpeed(1.0)
        } else {
            Indexer.setSpeed(0.0)
        }
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(7.0)
    }

    override fun getRequirements() = setOf(Indexer)

}
