package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive

class RightTrajectories(private val drive: SampleMecanumDrive) {
    val toMid =
        drive.trajectoryBuilder(Pose2d(0.0, 0.0))
            .lineToSplineHeading(Pose2d(34.0, 4.0, Math.toRadians(45.0)))
            .build()
}
