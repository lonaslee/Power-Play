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
    private val Int.rad get() = Math.toRadians(this.toDouble())

    companion object {
        @JvmField var x1 = -40.0
        @JvmField var y1 = -27.0
        @JvmField var h1 = 25.0

        @JvmField var x2 = -40.0
        @JvmField var y2 = -27.0
        @JvmField var h2 = 25.0

        val pose1 get() = Pose2d(x1, y1, Math.toRadians(h1))
        val pose2 get() = Pose2d(x2, y2, Math.toRadians(h2))

        fun Pose2d(x: Int, y: Int, heading: Int) =
            Pose2d(x.toDouble(), y.toDouble(), heading.toDouble())
    }

    val traj = drive.trajectorySequenceBuilder(Pose2d())
        .addTemporalMarker { arm.height = MID }
        .splineToLinearHeading(Pose2d(32.0, -3.0, (-40).rad), 0.rad)
        .addTemporalMarker { claw.open() }
        .waitSeconds(0.2)

        .UNSTABLE_addTemporalMarkerOffset(0.4) { arm.height = STACK }
        .lineToSplineHeading(Trajectories.Pose2d(48, 5 , 45))
        .splineTo(Trajectories.Vector2d(50, 30), 90.rad)

        .build()!!


    init {
        drive.followTrajectorySequenceAsync(traj)
    }
}
