package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.subsystems.Arm
import org.firstinspires.ftc.teamcode.robot.subsystems.Claw
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline

@Config
class LeftTraj(
    private val drive: SampleMecanumDrive, private val arm: Arm, private val claw: Claw
) {
    companion object {
        val Int.rad get() = Math.toRadians(this.toDouble())
        fun Pose2d(x: Int, y: Int, heading: Int) = Pose2d(x.toDouble(), y.toDouble(), heading.rad)
    }

    fun byTag(tag: AprilTagPipeline.Tag) = when (tag) {
        AprilTagPipeline.Tag.LEFT  -> left
        AprilTagPipeline.Tag.RIGHT -> right
        else                       -> middle
    }

    private val coneTraj = drive.trajectorySequenceBuilder(Pose2d())
        .addTemporalMarker { arm.state = Arm.MID }
        .splineToLinearHeading(Pose2d(31, -3, -40), 0.rad)
        .waitSeconds(0.2)
        .addTemporalMarker { claw.open() }
        .waitSeconds(0.2)
        .UNSTABLE_addTemporalMarkerOffset(0.3) { arm.state = Arm.STACK }
        .strafeLeft(10.0)
        // to stack
        .splineToLinearHeading(Pose2d(55, 10, 90), 0.rad)
        .forward(10.5)
        .waitSeconds(0.1)
        .addTemporalMarker { claw.close() }
        .waitSeconds(0.2)
        .addTemporalMarker { arm.state = Arm.MID }
        .lineToSplineHeading(Pose2d(49, -5, 55))
        .addTemporalMarker { arm.state = Arm.BACKMID }
        .waitSeconds(1.5)
        .addTemporalMarker { claw.open() }
        .waitSeconds(0.2)
        .addTemporalMarker { arm.state = Arm.STACK }
        // to stack 2
        .splineToLinearHeading(Pose2d(55, 10, 90), 0.rad)
        .forward(10.5)
        .waitSeconds(0.1)
        .addTemporalMarker { claw.close() }
        .waitSeconds(0.2)
        .addTemporalMarker { arm.state = Arm.MID }
        .lineToSplineHeading(Pose2d(49, -5, 55))
        .addTemporalMarker { arm.state = Arm.BACKMID }
        .waitSeconds(1.5)
        .addTemporalMarker { claw.open() }
        .waitSeconds(0.2)
        .addTemporalMarker { arm.state = Arm.STACK }
        // to stack 3
        .splineToLinearHeading(Pose2d(55, 10, 90), 0.rad)
        .forward(10.5)
        .waitSeconds(0.1)
        .addTemporalMarker { claw.close() }
        .waitSeconds(0.2)
        .addTemporalMarker { arm.state = Arm.MID }
        .lineToSplineHeading(Pose2d(49, -5, 55))
        .addTemporalMarker { arm.state = Arm.BACKMID }
        .waitSeconds(1.5)
        .addTemporalMarker { claw.open() }
        .waitSeconds(0.2)

    val left = coneTraj.lineToLinearHeading(Pose2d(53, 24, 0))
        .addTemporalMarker { arm.state = Arm.GROUND }
        .build()!!
    val right = coneTraj.lineToLinearHeading(Pose2d(53, -24, 0))
        .addTemporalMarker { arm.state = Arm.GROUND }
        .build()!!
    val middle = coneTraj.lineToLinearHeading(Pose2d(53, 0, 0))
        .addTemporalMarker { arm.state = Arm.GROUND }
        .build()!!
}
