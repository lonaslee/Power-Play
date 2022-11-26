package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.Arm3
import org.firstinspires.ftc.teamcode.robot.Arm3.Height
import org.firstinspires.ftc.teamcode.robot.Claw

private val Int.rad get() = Math.toRadians(this.toDouble())

class Trajectories(drive: SampleMecanumDrive, arm: Arm3, claw: Claw) {
    val left = drive.trajectorySequenceBuilder(leftStartPos)
        // to middle junction
        .addTemporalMarker { arm.height = Height.MID }
        .lineToSplineHeading(Pose2d(-32, -32, 45))
        .addTemporalMarker { println("OPEN CLAW");claw.open() }

        // to stack
        .addTemporalMarker { arm.height = Height.GROUND }
        .lineToSplineHeading(Pose2d(-36, -24, 116))
        .splineTo(Vector2d(-56, -12), 180.rad)

        // grab 1
        .addTemporalMarker { println("CLOSE CLAW");claw.close() }

        // to middle junction
        .addTemporalMarker { arm.height = Height.MID }
        .lineToSplineHeading(Pose2d(-32, -12, -45))
        .addTemporalMarker { println("OPEN CLAW");claw.open() }

        // to stack
        .addTemporalMarker { arm.height = Height.GROUND }
        .lineToSplineHeading(Pose2d(-56, -12, 180))

        // grab 2
        .addTemporalMarker { println("CLOSE CLAW");claw.close() }

        // to middle junction
        .addTemporalMarker { arm.height = Height.MID }
        .lineToSplineHeading(Pose2d(-32, -12, -45))
        .addTemporalMarker { println("OPEN CLAW");claw.open() }

        // to stack
        .addTemporalMarker { arm.height = Height.GROUND }
        .lineToSplineHeading(Pose2d(-56, -12, 180))

        // grab 3
        .addTemporalMarker { println("CLOSE CLAW");claw.close() }

        // to middle junction
        .addTemporalMarker { arm.height = Height.MID }
        .lineToSplineHeading(Pose2d(-32, -12, -45))
        .addTemporalMarker { println("OPEN CLAW");claw.open() }

        // to stack
        .addTemporalMarker { arm.height = Height.GROUND }
        .lineToSplineHeading(Pose2d(-56, -12, 180))

        // grab 4
        .addTemporalMarker { println("CLOSE CLAW");claw.close() }

        // to middle junction
        .addTemporalMarker { arm.height = Height.MID }
        .lineToSplineHeading(Pose2d(-32, -12, -45))
        .addTemporalMarker { println("OPEN CLAW");claw.open() }

        // to stack
        .addTemporalMarker { arm.height = Height.GROUND }
        .lineToSplineHeading(Pose2d(-56, -12, 180))

        // grab 5
        .addTemporalMarker { println("CLOSE CLAW");claw.close() }

        // to middle junction
        .addTemporalMarker { arm.height = Height.MID }
        .lineToSplineHeading(Pose2d(-32, -12, -45))
        .addTemporalMarker { println("OPEN CLAW");claw.open() }

        .addTemporalMarker { arm.height = Height.GROUND }
        .lineToSplineHeading(Pose2d(-36, -36, 270))
        .lineToConstantHeading(Vector2d(-12, -36))

        .build()!! // so excited!! NullPointerException!! lets go!!

    companion object {
        val leftStartPos = Pose2d(-36, -60, 90)

        fun Pose2d(x: Int = 0, y: Int = 0, heading: Int = 0) =
            Pose2d(x.toDouble(), y.toDouble(), Math.toRadians(heading.toDouble()))

        fun Vector2d(x: Int = 0, y: Int = 0) = Vector2d(x.toDouble(), y.toDouble())
    }
}
