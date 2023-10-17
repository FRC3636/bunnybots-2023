package frc.robot.commands

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Turret.Turret
import frc.robot.subsystems.vision.Vision
import frc.robot.subsystems.vision.Vision.Measurement
import frc.robot.utils.LimelightHelpers.LimelightTarget_Retro

class TrackClosest(private val turret: Turret,private val vision: Vision) : CommandBase(){


    var currentTarget: LimelightTarget_Retro = vision.closestTarget

    private var mode: AimMode = AimMode.SEARCH



    init{
        addRequirements(turret)
    }

    override fun execute(){
        if(!vision.hasTargets){ mode = AimMode.SEARCH}

        if(mode == AimMode.TRACK){
            currentTarget = vision.closestTarget
            TargetSamples.addMeasurement(vision.takeMeasurement(currentTarget))
            Turret.setTarget(Turret.relativeAngle.plus(Rotation2d.fromDegrees(currentTarget.tx)))
        }else if(mode == AimMode.SEARCH){
            TargetSamples.reset()

        }

        //TODO figure out if timestamping is fucked

    }


    internal companion object{
        const val SAMPLE_NUM = 8
        const val LEAD_TIME_MS = 300
    }



     enum class AimMode{
        TRACK,
        SEARCH,
    }

}

class AimAtTarget : CommandBase(){

}

object TargetSamples {

    private var samples: Array<Measurement?> = Array(TrackClosest.SAMPLE_NUM){ null }

    fun addMeasurement(measurement: Measurement){

        samples = samples.mapIndexed { index, _ -> if(index != 7) { samples[index+1]} else {measurement}}.toTypedArray()
    }

    fun reset(){
        samples = Array(TrackClosest.SAMPLE_NUM){ null }
    }

}

