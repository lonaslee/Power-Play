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
        .forward(48.0)
        .splineToLinearHeading(Pose2d(50, -10, 45), 0.rad)

        .build()


    companion object {
        @JvmField var x = 50.0
        @JvmField var y = -10.0
        @JvmField var heading = 45

        val aPose get() = Pose2d(x, y, heading.rad)

        val Int.rad get() = Math.toRadians(this.toDouble())

        fun Pose2d(x: Int, y: Int, heading: Int) = Pose2d(x.toDouble(), y.toDouble(), heading.rad)
    }
}