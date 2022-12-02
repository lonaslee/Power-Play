package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.Arm
import org.firstinspires.ftc.teamcode.robot.Arm.Height.*
import org.firstinspires.ftc.teamcode.robot.Claw

@Config
class LeftTraj(
    private val drive: SampleMecanumDrive, private val arm: Arm, private val claw: Claw
) {
    companion object {
        @JvmField var a_x = 48.0
        @JvmField var a_y = -5.0
        @JvmField var a_h = 55.0

        @JvmField var b_x = 51.0
        @JvmField var b_y = -5.0
        @JvmField var b_h = 55.0

        @JvmField var d = 11.5
        @JvmField var s = 10.0

        val Int.rad get() = Math.toRadians(this.toDouble())

        val aPose get() = Pose2d(a_x, a_y, Math.toRadians(a_h))
        val bPose get() = Pose2d(b_x, b_y, Math.toRadians(b_h))

        fun Pose2d(x: Int, y: Int, heading: Int) = Pose2d(x.toDouble(), y.toDouble(), heading.rad)
    }

    val traj = drive.trajectorySequenceBuilder(Pose2d())
        .addTemporalMarker { arm.height = MID }
        .splineToLinearHeading(Pose2d(31, -3, -40), 0.rad)
        .waitSeconds(0.2)
        .addTemporalMarker { claw.open() }
        .waitSeconds(0.2)
        .UNSTABLE_addTemporalMarkerOffset(0.3) { arm.height = STACK }
        .strafeLeft(s)
        // to stack
        .splineToLinearHeading(Pose2d(55, 10, 90), 0.rad)
        .forward(10.5)
        .waitSeconds(0.1)
        .addTemporalMarker { claw.close() }
        .waitSeconds(0.2)
        .addTemporalMarker { arm.height = MID }
        .lineToSplineHeading(aPose)
        .addTemporalMarker { arm.height = BACKMID }
        .waitSeconds(1.5)
        .addTemporalMarker { claw.open() }
        .waitSeconds(0.2)
        .addTemporalMarker { arm.height = STACK }
        // to stack 2
        .splineToLinearHeading(Pose2d(55, 10, 90), 0.rad)
        .forward(10.5)
        .waitSeconds(0.1)
        .addTemporalMarker { claw.close() }
        .waitSeconds(0.2)
        .addTemporalMarker { arm.height = MID }
        .lineToSplineHeading(aPose)
        .addTemporalMarker { arm.height = BACKMID }
        .waitSeconds(1.5)
        .addTemporalMarker { claw.open() }
        .waitSeconds(0.2)
        .addTemporalMarker { arm.height = STACK }
        // to stack 3
        .splineToLinearHeading(Pose2d(57, 10, 90), 0.rad)
        .forward(10.5)
        .waitSeconds(0.1)
        .addTemporalMarker { claw.close() }
        .waitSeconds(0.2)
        .addTemporalMarker { arm.height = MID }
        .lineToSplineHeading(bPose)
        .addTemporalMarker { arm.height = BACKMID }
        .waitSeconds(1.5)
        .addTemporalMarker { claw.open() }
        .waitSeconds(0.2)

        .lineToLinearHeading(Pose2d(28, 0, 90))
        .addTemporalMarker { arm.height = GROUND }
        .back(28.0)

        .build()!!


    init {
        drive.followTrajectorySequenceAsync(traj)
    }
}
