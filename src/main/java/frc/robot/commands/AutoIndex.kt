package frc.robot.commands

import edu.wpi.first.wpilibj2.command.*
import frc.robot.subsystems.indexer.Indexer

class AutoIndex : SequentialCommandGroup(
    object : Command {
        override fun isFinished(): Boolean = Indexer.objectDetected
        override fun getRequirements() = emptySet<Subsystem>()
    },
    object : SequentialCommandGroup(
        InstantCommand({
            Indexer.setSpeed(1.0)
        }),
        WaitCommand(7.0),
        InstantCommand({
            Indexer.setSpeed(0.0)
        }),
    ) {
        override fun getRequirements() = setOf(Indexer)
    }
)
