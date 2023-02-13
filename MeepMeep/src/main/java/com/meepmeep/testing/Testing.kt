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
    var aX = -28.0
    var aY = -8.0
    var aH = 48
    var bX = -54.5
    var bY = -15.0
    var bH = 180
    var t = 0.2
    var d = 0.1
    var pickWait = 0.3
    var raiseWait = 0.3
    var dropWait = 0.3
    var w = 0.2
    var acc = 30.0
    var abb = 25.0

    val dropVec = Vector2d(aX, aY)
    val pickVec = Vector2d(bX, bY)

    val startPose = Pose2d(-31.0, -61.0, (-90).rad)


    fun Pose2d(x: Int, y: Int, heading: Int) = Pose2d(x.toDouble(), y.toDouble(), heading.rad)
    fun Vector2d(x: Int, y: Int) = Vector2d(x.toDouble(), y.toDouble())

    bot.followTrajectorySequence(
        drive.trajectorySequenceBuilder(Pose2d(-31.0, -61.0, (-90).rad))
            .setReversed(true)
            .splineTo(Vector2d(-32.0, -9.0), 45.rad)
            .setReversed(false)
            .forward(5.0)
            .turn(45.0)
            .build()
    )


    // park
//            .splineTo(Vector2d(-36, -12), 270.rad)
//            .splineToSplineHeading(Pose2d(-12, -12, 270), 0.rad)
//            .splineToSplineHeading(Pose2d(-60, -12, 270), 180.rad)

//            .waitSeconds(0.5)
//            .build()!!)


    meepmeep.addEntity(bot).start()
}

val Int.rad get() = this.toDouble().toRadians()
fun Vector2d(x: Int, y: Int) = Vector2d(x.toDouble(), y.toDouble())
fun Pose2d(x: Int, y: Int, h: Int = 0) = Pose2d(x.toDouble(), y.toDouble(), h.rad)