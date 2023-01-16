package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Claw


@com.acmerobotics.dashboard.config.Config
class LeftTrajectory(drive: SampleMecanumDrive, arm: Arm, claw: Claw) {
    val initPoseTraj = drive.trajectorySequenceBuilder(Pose2d())
        .splineToSplineHeading(aPose, aH)
        .waitSeconds(5.0)
        .splineToLinearHeading(bPose, bH)
        .waitSeconds(5.0)
        .lineToSplineHeading(cPose)
        .build()!!

    companion object {
        @JvmField var aX = 53.0
        @JvmField var aY = 6.0
        @JvmField var aD = 135
        @JvmField var aH = -5.2

        @JvmField var bX = 48.0
        @JvmField var bY = -20.0
        @JvmField var bD = 90
        @JvmField var bH = 90.rad

        @JvmField var cX = 53.0
        @JvmField var cY = 6.0
        @JvmField var cD = 135

        val aPose get() = Pose2d(aX, aY, aD.rad)
        val bPose get() = Pose2d(bX, bY, bD.rad)
        val cPose get() = Pose2d(cX, cY, cD.rad)

        val Int.rad get() = Math.toRadians(this.toDouble())

        fun Pose2d(x: Int, y: Int, heading: Int) = Pose2d(x.toDouble(), y.toDouble(), heading.rad)
    }
}
