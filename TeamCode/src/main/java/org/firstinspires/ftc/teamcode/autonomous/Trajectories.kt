package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.Arm
import org.firstinspires.ftc.teamcode.robot.Arm.Height.*
import org.firstinspires.ftc.teamcode.robot.Claw


class Trajectories(drive: SampleMecanumDrive, arm: Arm, claw: Claw) {
    private val Int.rad get() = Math.toRadians(this.toDouble())

    val leftback = drive.trajectorySequenceBuilder(leftStartPos)
        .addTemporalMarker { arm.state = MID }
        .lineToSplineHeading(Pose2d(-36, -40, 45))
        .addTemporalMarker { claw.open() }

        // to stack
        .addTemporalMarker { arm.state = STACK }
        .lineToSplineHeading(Pose2d(-36, -24, 116))
        .splineTo(Vector2d(-56, -12), 180.rad)

        // grab 1
        .addTemporalMarker { claw.close() }
        .waitSeconds(0.3)

        // to middle junction
        .addTemporalMarker { arm.state = BACKMID }
        .waitSeconds(0.6)
        .lineToSplineHeading(Pose2d(-36, -25, 27))
        .addTemporalMarker { claw.open() }
        .waitSeconds(0.4)

        // to stack
        .addTemporalMarker { arm.state = STACK }
        .lineToSplineHeading(Pose2d(-56, -12, 180))

        // repeat grabs

        // grab 1
        .addTemporalMarker { claw.close() }
        .waitSeconds(0.3)

        // to middle junction
        .addTemporalMarker { arm.state = BACKMID }
        .waitSeconds(0.6)
        .lineToSplineHeading(Pose2d(-32, -12, 135))
        .addTemporalMarker { claw.open() }
        .waitSeconds(0.4)

        // to stack
        .addTemporalMarker { arm.state = STACK }
        .lineToSplineHeading(Pose2d(-56, -12, 180))
        // grab 1
        .addTemporalMarker { claw.close() }
        .waitSeconds(0.3)

        // to middle junction
        .addTemporalMarker { arm.state = BACKMID }
        .waitSeconds(0.6)
        .lineToSplineHeading(Pose2d(-32, -12, 135))
        .addTemporalMarker { claw.open() }
        .waitSeconds(0.4)

        // to stack
        .addTemporalMarker { arm.state = STACK }
        .lineToSplineHeading(Pose2d(-56, -12, 180))
        // grab 1
        .addTemporalMarker { claw.close() }
        .waitSeconds(0.3)

        // to middle junction
        .addTemporalMarker { arm.state = BACKMID }
        .waitSeconds(0.6)
        .lineToSplineHeading(Pose2d(-32, -12, 135))
        .addTemporalMarker { claw.open() }
        .waitSeconds(0.4)

        // to stack
        .addTemporalMarker { arm.state = STACK }
        .lineToSplineHeading(Pose2d(-56, -12, 180))
        // grab 1
        .addTemporalMarker { claw.close() }
        .waitSeconds(0.3)

        // to middle junction
        .addTemporalMarker { arm.state = BACKMID }
        .waitSeconds(0.6)
        .lineToSplineHeading(Pose2d(-32, -12, 135))
        .addTemporalMarker { claw.open() }
        .waitSeconds(0.4)

        // to stack
        .addTemporalMarker { arm.state = STACK }
        .lineToSplineHeading(Pose2d(-56, -12, 180))

        // park
        .lineToLinearHeading(Pose2d(-36, -36, 90))
        .lineToConstantHeading(Vector2d(-12, -36))

        .build()!!


    val left = drive.trajectorySequenceBuilder(leftStartPos)
        // to middle junction
        .addTemporalMarker { arm.state = MID }
        .lineToSplineHeading(Pose2d(-32, -32, 45))
        .addTemporalMarker { println("OPEN CLAW");claw.open() }
        .waitSeconds(0.1)

        // to stack
        .addTemporalMarker { arm.state = STACK }
        .lineToSplineHeading(Pose2d(-36, -24, 116))
        .splineTo(Vector2d(-56, -12), 180.rad)

        // grab 1
        .addTemporalMarker { println("CLOSE CLAW");claw.close() }
        .waitSeconds(0.15)

        // to middle junction
        .addTemporalMarker { arm.state = MID }
        .waitSeconds(0.3)
        .lineToSplineHeading(Pose2d(-32, -12, -45))
        .addTemporalMarker { println("OPEN CLAW");claw.open() }
        .waitSeconds(0.1)

        // to stack
        .addTemporalMarker { arm.state = STACK }
        .lineToSplineHeading(Pose2d(-56, -12, 180))

        // grab 2
        .addTemporalMarker { println("CLOSE CLAW");claw.close() }
        .waitSeconds(0.15)

        // to middle junction
        .addTemporalMarker { arm.state = MID }
        .waitSeconds(0.3)
        .lineToSplineHeading(Pose2d(-32, -12, -45))
        .addTemporalMarker { println("OPEN CLAW");claw.open() }
        .waitSeconds(0.1)

        // to stack
        .addTemporalMarker { arm.state = STACK }
        .lineToSplineHeading(Pose2d(-56, -12, 180))

        // grab 3
        .addTemporalMarker { println("CLOSE CLAW");claw.close() }
        .waitSeconds(0.15)

        // to middle junction
        .addTemporalMarker { arm.state = MID }
        .waitSeconds(0.3)
        .lineToSplineHeading(Pose2d(-32, -12, -45))
        .addTemporalMarker { println("OPEN CLAW");claw.open() }
        .waitSeconds(0.1)

        // to stack
        .addTemporalMarker { arm.state = STACK }
        .lineToSplineHeading(Pose2d(-56, -12, 180))

        // grab 4
        .addTemporalMarker { println("CLOSE CLAW");claw.close() }
        .waitSeconds(0.15)

        // to middle junction
        .addTemporalMarker { arm.state = MID }
        .waitSeconds(0.3)
        .lineToSplineHeading(Pose2d(-32, -12, -45))
        .addTemporalMarker { println("OPEN CLAW");claw.open() }
        .waitSeconds(0.1)

        // to stack
        .addTemporalMarker { arm.state = STACK }
        .lineToSplineHeading(Pose2d(-56, -12, 180))

        // grab 5
        .addTemporalMarker { println("CLOSE CLAW");claw.close() }
        .waitSeconds(0.15)

        // to middle junction
        .addTemporalMarker { arm.state = MID }
        .waitSeconds(0.3)
        .lineToSplineHeading(Pose2d(-32, -12, -45))
        .addTemporalMarker { println("OPEN CLAW");claw.open() }
        .waitSeconds(0.1)

        .addTemporalMarker { arm.state = STACK }
        .lineToSplineHeading(Pose2d(-36, -36, 270))
        .lineToConstantHeading(Vector2d(-12, -36))

        .build()!! // so excited!! NullPointerException!! lets go!!

    companion object {
        val leftStartPos = Pose2d(-36, -66, 90)

        fun Pose2d(x: Int = 0, y: Int = 0, heading: Int = 0) =
            Pose2d(x.toDouble(), y.toDouble(), Math.toRadians(heading.toDouble()))

        fun Vector2d(x: Int = 0, y: Int = 0) = Vector2d(x.toDouble(), y.toDouble())
    }
}
