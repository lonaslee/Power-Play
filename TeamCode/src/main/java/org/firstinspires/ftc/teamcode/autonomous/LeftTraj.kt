package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.config.Config
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

@Config
class LeftTraj(
    private val drive: SampleMecanumDrive, private val arm: Arm, private val claw: Claw
) {
    companion object {
        @JvmField var a_x = 49.0
        @JvmField var a_y = -5.0
        @JvmField var a_h = 55.0

        @JvmField var b_x = 49.0
        @JvmField var b_y = -5.0
        @JvmField var b_h = 55.0

        @JvmField var d = 11.5
        @JvmField var s = 10.0

        val endPose = Pose2d(55, 10, 90)

        val Int.rad get() = Math.toRadians(this.toDouble())

        val aPose get() = Pose2d(a_x, a_y, Math.toRadians(a_h))
        val bPose get() = Pose2d(b_x, b_y, Math.toRadians(b_h))

        fun Pose2d(x: Int, y: Int, heading: Int) = Pose2d(x.toDouble(), y.toDouble(), heading.rad)
    }

    fun byTag(tag: Tag) = when (tag) {
        Tag.LEFT  -> left
        Tag.RIGHT -> right
        else      -> middle
    }

    private val coneTraj
        get() = drive.trajectorySequenceBuilder(Pose2d())
            .addTemporalMarker { claw.state = CLOSED }
            .waitSeconds(0.2)
            .addTemporalMarker { arm.state = MID }
            .splineToLinearHeading(Pose2d(31, -3, -40), 0.rad)
            .waitSeconds(0.2)
            .addTemporalMarker { claw.state = OPENED }
            .waitSeconds(0.2)
            .UNSTABLE_addTemporalMarkerOffset(0.3) { arm.state = STACK }
            .strafeLeft(s)
            // to stack
            .splineToLinearHeading(Pose2d(55, 10, 90), 0.rad)
            .forward(10.5)
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
            // to stack 2
            .splineToLinearHeading(Pose2d(55, 10, 90), 0.rad)
            .forward(10.5)
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
            .forward(10.5)
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

    val left = coneTraj.lineToLinearHeading(Pose2d(53, 24, 0)).build()!!
    val right = coneTraj.lineToLinearHeading(Pose2d(53, -24, 0)).build()!!
    val middle = coneTraj.lineToLinearHeading(Pose2d(53, 0, 0)).build()!!
}
