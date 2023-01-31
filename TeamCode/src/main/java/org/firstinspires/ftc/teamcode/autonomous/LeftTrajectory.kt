package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Arm3
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.vision.SignalSleevePipeline


@com.acmerobotics.dashboard.config.Config
class LeftTrajectory(val drive: SampleMecanumDrive, val arm: Arm3, val claw: Claw) {
    private val coneTraj
        get() = drive.trajectorySequenceBuilder(startPose)
            .setReversed(true)
            .addTemporalMarker { arm.state = Arm.BACKHIGH }
            .splineTo(Vector2d(-28, -5), 55.rad)
            .waitSeconds(0.2)
            .addTemporalMarker { claw.state = Claw.OPENED }
            .waitSeconds(t)
            .addTemporalMarker { arm.state = Arm.STACK }
            .setReversed(false)
            // pick 1
            .setAccelConstraint { _, _, _, _ -> mAV.toDouble() }
            .splineTo(pickVec, (bH).rad)
            .addTemporalMarker { claw.state = Claw.CLOSED }
            .waitSeconds(pickWait)
            .addTemporalMarker { arm.state = Arm.HIGH }
            .waitSeconds(raiseWait)
            .setAccelConstraint { _, _, _, _ -> mAV.toDouble() }
            .setReversed(true)
            .splineTo(dropVec, aH.rad)
            .addTemporalMarker { arm.state = Arm.BACKHIGH }
            .waitSeconds(0.3)
            .waitSeconds(dropWait)
            .addTemporalMarker { claw.state = Claw.OPENED }
            .waitSeconds(t)
            .addTemporalMarker { arm.state = Arm.STACK }
            .setReversed(false)
//            // pick 2
//            .splineTo(pickVec, (bH).rad)
//            .addTemporalMarker { claw.state = Claw.CLOSED }
//            .waitSeconds(pickWait)
//            .addTemporalMarker { arm.state = Arm.BACKHIGH }
//            .waitSeconds(raiseWait)
//            .setReversed(true)
//            .splineTo(dropVec, aH.rad)
//            .waitSeconds(dropWait)
//            .addTemporalMarker { claw.state = Claw.OPENED }
//            .waitSeconds(t)
//            .addTemporalMarker { arm.state = Arm.STACK }
//            .setReversed(false)
//            // pick 3
//            .splineTo(pickVec, (bH).rad)
//            .addTemporalMarker { claw.state = Claw.CLOSED }
//            .waitSeconds(pickWait)
//            .addTemporalMarker { arm.state = Arm.BACKHIGH }
//            .waitSeconds(raiseWait)
//            .setReversed(true)
//            .splineTo(dropVec, aH.rad)
//            .waitSeconds(dropWait)
//            .addTemporalMarker { claw.state = Claw.OPENED }
//            .waitSeconds(t)
//            .addTemporalMarker { arm.state = Arm.STACK }
//            .setReversed(false)

            .addTemporalMarker { arm.state = Arm.GROUND }

    val middle = coneTraj.splineTo(Vector2d(-36.0, -11.5), 270.rad).build()!!
    val left = coneTraj.splineToSplineHeading(Pose2d(-60.0, -11.5, 270.rad), 180.rad).build()!!
    val right = coneTraj.splineToSplineHeading(Pose2d(-12.0, -11.5, 270.rad), 0.rad).build()!!

    fun byTag(tag: SignalSleevePipeline.Tag) = when (tag) {
        SignalSleevePipeline.Tag.LEFT -> left
        SignalSleevePipeline.Tag.RIGHT -> right
        else -> middle
    }

    companion object {
        @JvmField var aX = -28
        @JvmField var aY = -8
        @JvmField var aH = 55

        @JvmField var bX = -58
        @JvmField var bY = -16
        @JvmField var bH = 180

        @JvmField var t = 0.3
        @JvmField var pickWait = 0.3
        @JvmField var raiseWait = 0.3
        @JvmField var dropWait = 0.3

        @JvmField var mAV = 30

        val dropVec get() = Vector2d(aX, aY)
        val pickVec get() = Vector2d(bX, bY)

        val startPose = Pose2d(-31.0, -61.0, (-90).rad)

        val Int.rad get() = Math.toRadians(this.toDouble())

        fun Pose2d(x: Int, y: Int, heading: Int) = Pose2d(x.toDouble(), y.toDouble(), heading.rad)
        fun Vector2d(x: Int, y: Int) = Vector2d(x.toDouble(), y.toDouble())
    }
}
