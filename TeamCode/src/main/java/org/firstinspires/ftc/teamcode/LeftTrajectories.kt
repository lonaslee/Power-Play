package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive

class LeftTrajectories(private val drive: SampleMecanumDrive) {
    val toMid = drive.trajectoryBuilder(Pose2d(0.0, 0.0))
        .lineToSplineHeading(Pose2d(30.0, -4.0, Math.toRadians(-45.0))).build()

    val toStack = drive.trajectorySequenceBuilder(toMid.end()).waitSeconds(0.5)
        .splineTo(Vector2d(32.0, 0.0), Math.toRadians(0.0))
        .splineTo(Vector2d(52.0, 26.0), Math.toRadians(90.0)).build()!!
}
