package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.vision.SignalSleevePipeline


@com.acmerobotics.dashboard.config.Config
class LeftTrajectory(val drive: SampleMecanumDrive, val arm: Arm, val claw: Claw) {
    private val coneTraj
        get() = drive.trajectorySequenceBuilder(startPose)
            .setReversed(true)
            .addTemporalMarker { arm.state = Arm.BACKHIGH }
            .splineTo(dropVec, aH.rad)
            .waitSeconds(0.3)
            .addTemporalMarker { claw.state = Claw.OPENED }
            .waitSeconds(0.2)
            .addTemporalMarker { arm.state = Arm.STACK }
            .setReversed(false)
            // pick 1
            .splineTo(pickVec, (bH).rad)
            .waitSeconds(1.0)
            .setReversed(true)
            .splineTo(dropVec, aH.rad)
            .waitSeconds(0.5)
            .setReversed(false)
            // pick 2
            .splineTo(pickVec, (bH).rad)
            .waitSeconds(1.0)
            .setReversed(true)
            .splineTo(dropVec, aH.rad)
            .waitSeconds(0.5)
            .setReversed(false)
            // pick 3
            .splineTo(pickVec, (bH).rad)
            .waitSeconds(1.0)
            .setReversed(true)
            .splineTo(dropVec, aH.rad)
            .waitSeconds(0.5)
            .setReversed(false)
            // pick 4
            .splineTo(pickVec, (bH).rad)
            .waitSeconds(1.0)
            .setReversed(true)
            .splineTo(dropVec, aH.rad)
            .waitSeconds(0.5)
            .setReversed(false)
            // pick 5
            .splineTo(pickVec, (bH).rad)
            .waitSeconds(1.0)
            .setReversed(true)
            .splineTo(dropVec, aH.rad)
            .waitSeconds(0.5)
            .setReversed(false)

    val middle = coneTraj.splineTo(Vector2d(-36, -12), 270.rad).build()!!
    val left = coneTraj.splineToSplineHeading(Pose2d(-60, -12, 270), 180.rad).build()!!
    val right = coneTraj.splineToSplineHeading(Pose2d(-12, -12, 270), 0.rad).build()!!

    fun byTag(tag: SignalSleevePipeline.Tag) = when (tag) {
        SignalSleevePipeline.Tag.LEFT -> left
        SignalSleevePipeline.Tag.RIGHT -> right
        else -> middle
    }

    companion object {
        @JvmField var aX = -30
        @JvmField var aY = -6
        @JvmField var aH = 55

        @JvmField var bX = -58
        @JvmField var bY = -12
        @JvmField var bH = 180

        val dropVec get() = Vector2d(aX, aY)
        val pickVec get() = Vector2d(bX, bY)

        val startPose = Pose2d(-31.0, -61.0, (-90).rad)

        val Int.rad get() = Math.toRadians(this.toDouble())

        fun Pose2d(x: Int, y: Int, heading: Int) = Pose2d(x.toDouble(), y.toDouble(), heading.rad)
        fun Vector2d(x: Int, y: Int) = Vector2d(x.toDouble(), y.toDouble())
    }
}
