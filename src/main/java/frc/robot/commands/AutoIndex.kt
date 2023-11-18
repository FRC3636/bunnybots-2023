package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.indexer.Indexer

class AutoIndex : Command {
    override fun execute() {
        if(!Indexer.beamBreak) {
            Indexer.setSpeed(1.0)
        } else {
            Indexer.setSpeed(0.0)
        }
    }

    override fun getRequirements() = setOf(Indexer)

}
