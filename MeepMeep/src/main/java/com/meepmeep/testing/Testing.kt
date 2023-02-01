package com.meepmeep.testing

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.meepmeep.testing.LeftTrajectories.Companion.toRadians
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.toRadians
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder


val meepmeep = MeepMeep(800).apply {
    setBackground(Background.FIELD_POWERPLAY_OFFICIAL)
//    setDarkMode(true)
//    setBackgroundAlpha(0.95F)
}
val bot = DefaultBotBuilder(meepmeep).setConstraints(
    52.48291908330528, 52.48291908330528, 180.toRadians(), 180.toRadians(), 14.0
).setDimensions(16.0, 18.0).build()
val drive = bot.drive

val bot2 = DefaultBotBuilder(meepmeep).setConstraints(
    60.0, 60.0, 180.toRadians(), 180.toRadians(), 14.0
).setDimensions(16.0, 18.0).build()
val drive2 = bot2.drive

fun main() {
    bot.followTrajectorySequence(drive.trajectorySequenceBuilder(Pose2d(-31.0, -61.0, (-90).rad))
        .setReversed(true)
        .UNSTABLE_addTemporalMarkerOffset(1.0) { /* raise arm */ }
        .splineTo(Vector2d(-36, 0), 80.rad)
        .setReversed(false)
        .splineToLinearHeading(Pose2d(-28, -5, -135), 45.rad)
        .addTemporalMarker { /* open claw */ }
        .waitSeconds(1.0)

        .addTemporalMarker { /* lower arm */ }
        .splineTo(Vector2d(-58, -12), 180.rad)
        .addTemporalMarker { /* close claw */ }
        .waitSeconds(1.0)

        .setReversed(true)
        .addTemporalMarker { /* raise arm */ }
        .splineTo(Vector2d(-28, -5), 45.rad)
        .addTemporalMarker { /* open claw */ }
        .waitSeconds(1.0)

        .setReversed(false)
        .addTemporalMarker { /* lower arm */ }
        .splineTo(Vector2d(-58, -12), 180.rad)
        .addTemporalMarker { /* close claw */ }
        .waitSeconds(1.0)

        .setReversed(true)
        .addTemporalMarker { /* raise arm */ }
        .splineTo(Vector2d(-28, -5), 45.rad)
        .addTemporalMarker { /* open claw */ }
        .waitSeconds(1.0)

        .setReversed(false)
        .addTemporalMarker { /* lower arm */ }
        .splineTo(Vector2d(-58, -12), 180.rad)
        .addTemporalMarker { /* close claw */ }
        .waitSeconds(1.0)

        .setReversed(true)
        .addTemporalMarker { /* raise arm */ }
        .splineTo(Vector2d(-28, -5), 45.rad)
        .addTemporalMarker { /* open claw */ }
        .waitSeconds(1.0)

        .setReversed(false)
        .addTemporalMarker { /* lower arm */ }
        .splineTo(Vector2d(-58, -12), 180.rad)
        .addTemporalMarker { /* close claw */ }
        .waitSeconds(1.0)

        .setReversed(true)
        .addTemporalMarker { /* raise arm */ }
        .splineTo(Vector2d(-28, -5), 45.rad)
        .addTemporalMarker { /* open claw */ }
        .waitSeconds(1.0)

        .setReversed(false)
        .addTemporalMarker { /* lower arm */ }
        .splineTo(Vector2d(-58, -12), 180.rad)
        .addTemporalMarker { /* close claw */ }
        .waitSeconds(1.0)

        .setReversed(true)
        .addTemporalMarker { /* raise arm */ }
        .splineTo(Vector2d(-28, -5), 45.rad)
        .addTemporalMarker { /* open claw */ }
        .waitSeconds(1.0)


        .setReversed(false)
        // park
//            .splineTo(Vector2d(-36, -12), 270.rad)
//            .splineToSplineHeading(Pose2d(-12, -12, 270), 0.rad)
        .splineToSplineHeading(Pose2d(-60, -12, 270), 180.rad)

        .waitSeconds(0.5)
        .build()!!)

    meepmeep.addEntity(bot).start()
}

val Int.rad get() = this.toDouble().toRadians()
fun Vector2d(x: Int, y: Int) = Vector2d(x.toDouble(), y.toDouble())
fun Pose2d(x: Int, y: Int, h: Int = 0) = Pose2d(x.toDouble(), y.toDouble(), h.rad)