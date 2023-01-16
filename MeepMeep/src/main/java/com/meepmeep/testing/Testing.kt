package com.meepmeep.testing

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.meepmeep.testing.LeftTrajectories.Companion.toRadians
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.toRadians
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder


val meepmeep = MeepMeep(800).apply {
    setBackground(Background.FIELD_POWERPLAY_KAI_DARK)
    setDarkMode(true)
    setBackgroundAlpha(0.95F)
}
val bot = DefaultBotBuilder(meepmeep).setConstraints(
    60.0, 60.0, 180.toRadians(), 180.toRadians(), 14.0
).build()
val drive = bot.drive

fun main() {
    bot.followTrajectorySequence(
        LeftTrajectories(drive).zt
    )

    meepmeep.addEntity(bot).start()
}
