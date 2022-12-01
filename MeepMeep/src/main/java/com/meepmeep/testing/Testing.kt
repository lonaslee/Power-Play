package com.meepmeep.testing

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder


val meepmeep = MeepMeep(800).apply {
    setBackground(Background.FIELD_POWERPLAY_KAI_DARK)
    setDarkMode(true)
    setBackgroundAlpha(0.95F)
}
val bot = DefaultBotBuilder(meepmeep).setConstraints(60.0, 60.0, 180.rad, 180.rad, 14.0)
    .build()
val drive = bot.drive

fun main() {
    drive.poseEstimate = Pose2d(-36.0, -60.0, 90.0)
    bot.followTrajectorySequence(
        drive.trajectorySequenceBuilder(Pose2d())
//        .addTemporalMarker { arm.height = MID }
            .splineToLinearHeading(Pose2d(31.0, -3.0, (-40).rad), 0.rad)
            .waitSeconds(0.2)
//        .addTemporalMarker { claw.open() }
            .waitSeconds(0.2)
//        .UNSTABLE_addTemporalMarkerOffset(0.3) { arm.height = STACK }
            .strafeLeft(10.0)
            .splineTo(Vector2d(55.0, 10.0), Math.toRadians(90.0))

            .forward(12.0)
            .waitSeconds(0.2)
            .lineToLinearHeading(Pose2d(55.0, 10.0, 90.0))
//        .addTemporalMarker { claw.close() }
            .build()!!
    )

    meepmeep.addEntity(bot)
        .start()
}

val Int.rad get() = Math.toRadians(this.toDouble())