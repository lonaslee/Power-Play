package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.autonomous.LeftTraj.Companion.rad
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.subsystems.Arm
import org.firstinspires.ftc.teamcode.robot.subsystems.Claw
import org.firstinspires.ftc.teamcode.robot.subsystems.DriveExt

@com.acmerobotics.dashboard.config.Config
class LeftTrajectory(drive: SampleMecanumDrive, arm: Arm, claw: Claw) {
    val coneTraj = drive.trajectorySequenceBuilder(Pose2d())
        .addTemporalMarker { arm.state = Arm.MID }
        .splineToLinearHeading(Pose2d(31, -3, -40), 0.rad)
        .waitSeconds(0.2)
        .addTemporalMarker { claw.state = Claw.OPENED }
        .waitSeconds(0.2)
        .UNSTABLE_addTemporalMarkerOffset(0.3) { arm.state = Arm.STACK }
        .strafeLeft(10.0)
        // to stack
        .splineToLinearHeading(aPose, 0.rad)
        .waitSeconds(0.1)
        .addTemporalMarker { claw.state = Claw.CLOSED }
        .waitSeconds(0.2)
        .addTemporalMarker { arm.state = Arm.MID }
        .lineToSplineHeading(Pose2d(49, -5, 55))
        .addTemporalMarker { arm.state = Arm.BACKMID }
        .waitSeconds(1.5)
        .addTemporalMarker { claw.state = Claw.OPENED }
        .waitSeconds(0.2)
        .addTemporalMarker { arm.state = Arm.STACK }
        .build()


    companion object {
        @JvmField var x = 55.0
        @JvmField var y =20.5
        @JvmField var heading = 90

        val aPose get() = Pose2d(x, y, heading.rad)

        val Int.rad get() = Math.toRadians(this.toDouble())
        fun Pose2d(x: Int, y: Int, heading: Int) =
            com.acmerobotics.roadrunner.geometry.Pose2d(x.toDouble(), y.toDouble(), heading.rad)
    }
}