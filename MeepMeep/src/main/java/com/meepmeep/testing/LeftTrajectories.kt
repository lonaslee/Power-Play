package com.meepmeep.testing

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.roadrunner.DriveShim
import java.util.Vector


class LeftTrajectories(drive: DriveShim) {
    val trajectory = drive.trajectorySequenceBuilder(Pose2d(-36, -60, 90))
        // to middle junction
        .addTemporalMarker { println("RAISE ARM") }
        .lineToSplineHeading(Pose2d(-32, -32, 45))
        .waitSeconds(0.4)
        .addTemporalMarker { println("OPEN CLAW") }
        .waitSeconds(0.2)

        // to stack
        .addTemporalMarker { println("LOWER ARM") }
        .lineToSplineHeading(Pose2d(-36, -24, 116))
        .splineTo(Vector2d(-56, -12), 180.rad)

        // grab 1
        .addTemporalMarker { println("CLOSE CLAW") }
        .waitSeconds(0.4)

        // to middle junction
        .addTemporalMarker { println("RAISE ARM") }
        .lineToSplineHeading(Pose2d(-32, -12, -45))
        .waitSeconds(0.4)
        .addTemporalMarker { println("OPEN CLAW") }
        .waitSeconds(0.2)

        // to stack
        .addTemporalMarker { println("LOWER ARM") }
        .lineToSplineHeading(Pose2d(-56, -12, 180))

        // grab 2
        .addTemporalMarker { println("CLOSE CLAW") }
        .waitSeconds(0.4)

        // to middle junction
        .addTemporalMarker { println("RAISE ARM") }
        .lineToSplineHeading(Pose2d(-32, -12, -45))
        .waitSeconds(0.4)
        .addTemporalMarker { println("OPEN CLAW") }
        .waitSeconds(0.2)

        // to stack
        .addTemporalMarker { println("LOWER ARM") }
        .lineToSplineHeading(Pose2d(-56, -12, 180))

        // grab 3
        .addTemporalMarker { println("CLOSE CLAW") }
        .waitSeconds(0.4)

        // to middle junction 3
        .addTemporalMarker { println("RAISE ARM") }
        .lineToSplineHeading(Pose2d(-32, -12, -45))
        .waitSeconds(0.4)
        .addTemporalMarker { println("OPEN CLAW") }
        .waitSeconds(0.2)

        // to stack
        .addTemporalMarker { println("LOWER ARM") }
        .lineToSplineHeading(Pose2d(-56, -12, 180))

        // grab 4
        .addTemporalMarker { println("CLOSE CLAW") }
        .waitSeconds(0.4)

        // to middle junction
        .addTemporalMarker { println("RAISE ARM") }
        .lineToSplineHeading(Pose2d(-32, -12, -45))
        .waitSeconds(0.4)
        .addTemporalMarker { println("OPEN CLAW") }
        .waitSeconds(0.2)

        // to stack
        .addTemporalMarker { println("LOWER ARM") }
        .lineToSplineHeading(Pose2d(-56, -12, 180))

        // grab 5
        .addTemporalMarker { println("CLOSE CLAW") }
        .waitSeconds(0.4)

        // to middle junction
        .addTemporalMarker { println("RAISE ARM") }
        .lineToSplineHeading(Pose2d(-32, -12, -45))
        .waitSeconds(0.4)
        .addTemporalMarker { println("OPEN CLAW") }
        .waitSeconds(0.2)

        .splineTo(Vector2d(-36, -36), 270.rad)
        .lineToConstantHeading(Vector2d(-12, -36))

        .build()!! // so excited!! NullPointerException!! lets go!!

    companion object {
        fun Pose2d(x: Int = 0, y: Int = 0, heading: Int = 0) =
            Pose2d(x.toDouble(), y.toDouble(), heading.rad)

        fun Vector2d(x: Int = 0, y: Int = 0) = Vector2d(x.toDouble(), y.toDouble())
    }
}
