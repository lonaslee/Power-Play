package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Arm3
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.vision.SignalSleevePipeline
import java.lang.Math.toRadians

object Trajectories {
    fun generateLeft(drive: SampleMecanumDrive, arm: Arm3, claw: Claw) =
        LeftCycle(drive, arm, claw).let {
            Triple(
                it.new().splineToSplineHeading(
                    Pose2d(-60.0, -12.0, 270.rad), 180.rad
                ).build()!!,
                it.new().splineTo(Vector2d(-36, -12), 270.rad).build()!!,
                it.new().splineToSplineHeading(Pose2d(-12.0, -12.0, 270.rad), 0.rad).build()!!
            )
        }

    private class LeftCycle(val drive: SampleMecanumDrive, val arm: Arm3, val claw: Claw) {
        fun new() = drive.trajectorySequenceBuilder(Pose2d(-31.0, -61.0, (-90).rad))
            .setReversed(true)
            .UNSTABLE_addTemporalMarkerOffset(1.0) { arm.state = Arm.BACKHIGH }
            .splineTo(Vector2d(-36, 0), 80.rad)
            .setReversed(false)
            .splineToLinearHeading(Pose2d(-28.0, -5.0, (-135).rad), 45.rad)
            .addTemporalMarker { claw.state = Claw.OPENED }
            .waitSeconds(1.0)

            .addTemporalMarker { arm.state = Arm.STACK }
            .splineTo(Vector2d(-58, -12), 180.rad)
            .addTemporalMarker { claw.state = Claw.CLOSED }
            .waitSeconds(1.0)

            .setReversed(true)
            .addTemporalMarker { arm.state = Arm.BACKHIGH }
            .splineTo(Vector2d(-28, -5), 45.rad)
            .addTemporalMarker { claw.state = Claw.OPENED }
            .waitSeconds(1.0)

            .setReversed(false)
            .addTemporalMarker { arm.state = Arm.STACK }
            .splineTo(Vector2d(-58, -12), 180.rad)
            .addTemporalMarker { claw.state = Claw.CLOSED }
            .waitSeconds(1.0)

            .setReversed(true)
            .addTemporalMarker { arm.state = Arm.BACKHIGH }
            .splineTo(Vector2d(-28, -5), 45.rad)
            .addTemporalMarker { claw.state = Claw.OPENED }
            .waitSeconds(1.0)

            .setReversed(false)
            .addTemporalMarker { arm.state = Arm.STACK }
            .splineTo(Vector2d(-58, -12), 180.rad)
            .addTemporalMarker { claw.state = Claw.CLOSED }
            .waitSeconds(1.0)

            .setReversed(true)
            .addTemporalMarker { arm.state = Arm.BACKHIGH }
            .splineTo(Vector2d(-28, -5), 45.rad)
            .addTemporalMarker { claw.state = Claw.OPENED }
            .waitSeconds(1.0)

            .setReversed(false)
            .addTemporalMarker { arm.state = Arm.STACK }
            .splineTo(Vector2d(-58, -12), 180.rad)
            .addTemporalMarker { claw.state = Claw.CLOSED }
            .waitSeconds(1.0)

            .setReversed(true)
            .addTemporalMarker { arm.state = Arm.BACKHIGH }
            .splineTo(Vector2d(-28, -5), 45.rad)
            .addTemporalMarker { claw.state = Claw.OPENED }
            .waitSeconds(1.0)

            .setReversed(false)
            .addTemporalMarker { arm.state = Arm.STACK }
            .splineTo(Vector2d(-58, -12), 180.rad)
            .addTemporalMarker { claw.state = Claw.CLOSED }
            .waitSeconds(1.0)

            .setReversed(true)
            .addTemporalMarker { arm.state = Arm.BACKHIGH }
            .splineTo(Vector2d(-28, -5), 45.rad)
            .addTemporalMarker { claw.state = Claw.OPENED }
            .waitSeconds(1.0)

            .setReversed(false)
            .addTemporalMarker { arm.state = Arm.GROUND }!!
    }

    fun Triple<TrajectorySequence, TrajectorySequence, TrajectorySequence>.byTag(tag: SignalSleevePipeline.Tag) =
        when (tag) {
            SignalSleevePipeline.Tag.LEFT  -> first
            SignalSleevePipeline.Tag.RIGHT -> third
            else                           -> second
        }

    private fun Vector2d(x: Int, y: Int) = Vector2d(x.toDouble(), y.toDouble())
    private val Int.rad get() = toRadians(this.toDouble())
}
