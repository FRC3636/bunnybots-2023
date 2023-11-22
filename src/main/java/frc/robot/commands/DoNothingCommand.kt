package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem

class DoNothingCommand(private val requirement: Set<Subsystem>): Command {
    override fun getRequirements() = requirement
}