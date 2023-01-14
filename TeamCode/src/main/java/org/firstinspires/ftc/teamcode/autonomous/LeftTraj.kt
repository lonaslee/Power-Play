package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.subsystems.Arm
import org.firstinspires.ftc.teamcode.robot.subsystems.Arm.States.BACKMID
import org.firstinspires.ftc.teamcode.robot.subsystems.Arm.States.GROUND
import org.firstinspires.ftc.teamcode.robot.subsystems.Arm.States.MID
import org.firstinspires.ftc.teamcode.robot.subsystems.Arm.States.STACK
import org.firstinspires.ftc.teamcode.robot.subsystems.Claw
import org.firstinspires.ftc.teamcode.robot.subsystems.Claw.States.CLOSED
import org.firstinspires.ftc.teamcode.robot.subsystems.Claw.States.OPENED
import org.firstinspires.ftc.teamcode.vision.SignalSleevePipeline.Tag

@com.acmerobotics.dashboard.config.Config
class LeftTraj(
    private val drive: SampleMecanumDrive, private val arm: Arm, private val claw: Claw
) {
    companion object {
        @JvmField var o_x = 48.0
        @JvmField var o_y = -5.0
        @JvmField var o_h = 55.0

        @JvmField var a_x = 49.0
        @JvmField var a_y = -5.0
        @JvmField var a_h = 55.0

        @JvmField var b_x = 50.0
        @JvmField var b_y = -5.0
        @JvmField var b_h = 55.0

        @JvmField var d = 11.0

        val endPose = Pose2d(55, 10, 90)

        val Int.rad get() = Math.toRadians(this.toDouble())

        val aPose get() = Pose2d(a_x, a_y, Math.toRadians(a_h))
        val bPose get() = Pose2d(b_x, b_y, Math.toRadians(b_h))
        val oPose get() = Pose2d(o_x, o_y, Math.toRadians(o_h))

        fun Pose2d(x: Int, y: Int, heading: Int) = Pose2d(x.toDouble(), y.toDouble(), heading.rad)
    }

    fun byTag(tag: Tag) = drive.trajectorySequenceBuilder(Pose2d())
        .addTemporalMarker { arm.state = MID }
        .splineToLinearHeading(Pose2d(30, -3, -40), 0.rad)
        .waitSeconds(0.2)
        .addTemporalMarker { claw.state = OPENED }
        .waitSeconds(0.2)
        .UNSTABLE_addTemporalMarkerOffset(0.3) { arm.state = STACK }
        .strafeLeft(10.0)
        // to stack
        .splineToLinearHeading(Pose2d(55, 10, 90), 0.rad)
        .forward(d)
        .waitSeconds(0.1)
        .addTemporalMarker { claw.state = CLOSED }
        .waitSeconds(0.2)
        .addTemporalMarker { arm.state = MID }
        .lineToSplineHeading(oPose)
        .addTemporalMarker { arm.state = BACKMID }
        .waitSeconds(1.5)
        .addTemporalMarker { claw.state = OPENED }
        .waitSeconds(0.2)
        .addTemporalMarker { arm.state = STACK }
        // to stack 2
        .splineToLinearHeading(Pose2d(55, 10, 90), 0.rad)
        .forward(d)
        .waitSeconds(0.1)
        .addTemporalMarker { claw.state = CLOSED }
        .waitSeconds(0.2)
        .addTemporalMarker { arm.state = MID }
        .lineToSplineHeading(aPose)
        .addTemporalMarker { arm.state = BACKMID }
        .waitSeconds(1.5)
        .addTemporalMarker { claw.state = OPENED }
        .waitSeconds(0.2)
        .addTemporalMarker { arm.state = STACK }
        // to stack 3
        .splineToLinearHeading(Pose2d(55, 10, 90), 0.rad)
        .forward(d)
        .waitSeconds(0.1)
        .addTemporalMarker { claw.state = CLOSED }
        .waitSeconds(0.2)
        .addTemporalMarker { arm.state = MID }
        .lineToSplineHeading(bPose)
        .addTemporalMarker { arm.state = BACKMID }
        .waitSeconds(1.5)
        .addTemporalMarker { claw.state = OPENED }
        .waitSeconds(0.2)
        .addTemporalMarker { arm.state = GROUND }
        .lineToLinearHeading(
            when (tag) {
                Tag.LEFT -> Pose2d(53, 24, 0)
                Tag.RIGHT -> Pose2d(54, -24, 0)
                else -> Pose2d(53, 0, 0)
            }
        )
        .build()!!
}
