package com.meepmeep.testing

import com.acmerobotics.roadrunner.geometry.Pose2d
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
    bot.followTrajectorySequence(LeftTrajectories2(drive).trajectory)

    meepmeep.addEntity(bot)
        .start()
}

val Int.rad get() = Math.toRadians(this.toDouble())