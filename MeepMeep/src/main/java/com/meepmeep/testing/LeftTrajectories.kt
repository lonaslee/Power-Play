package com.meepmeep.testing

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.core.toRadians
import com.noahbres.meepmeep.roadrunner.DriveShim
import kotlin.math.atan2


class LeftTrajectories(drive: DriveShim) {
    val trajectory = drive.trajectorySequenceBuilder(initialPose)
        .splineToSplineHeading(Pose2d(6, 53, -45), 6.0)
        .splineToLinearHeading(Pose2d(-20, 48, 90), head(90))
        .lineToSplineHeading(Pose2d(6, 53, 135))
        .splineToLinearHeading(Pose2d(-20, 48, 90), head(90))

        .build()!!

    val zt = drive.trajectorySequenceBuilder(Pose2d())
        .splineToSplineHeading(Pose2d(53, 6, -135), -5.2)
        .splineToLinearHeading(Pose2d(48, -20, 90), head(90))
        .lineToSplineHeading(Pose2d(53, 6, 135))

        .build()!!

    companion object {
        val initialPose =/*Pose2d(0.0,0.0,90.0) */ Pose2d(-36.0, -60.0, 90.toRadians())

//        fun Pose2d(x: Int = 0, y: Int = 0, heading: Int = 0) =
//            Pose2d(x + initialPose.x, y + initialPose.y, heading.toRadians() + initialPose.heading)

        fun Pose2d(x: Int = 0, y: Int = 0, heading: Int = 0) =
            Pose2d(x.toDouble(), y.toDouble(), heading.toRadians())

        fun Vector2d(x: Int = 0, y: Int = 0) = Vector2d(x + initialPose.x, y + initialPose.y)

        fun head(a: Int) = a.toRadians() + initialPose.heading

        fun Int.toRadians() = toDouble().toRadians()
    }
}
