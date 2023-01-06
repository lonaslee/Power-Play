package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.subsystems.Arm
import org.firstinspires.ftc.teamcode.robot.subsystems.Claw
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline

private val AprilTagPipeline.Companion.Tag
    get() = object {
        val LEFT = 0
        val RIGHT = 1
    }


class Fonts {
    fun opModeIsActive() = false
    private lateinit var arm: Arm
    private lateinit var drive: SampleMecanumDrive
    private lateinit var aprilTagPipeline: AprilTagPipeline
    private lateinit var claw: Claw
    private var dir = 5

    object LeftTraj {
        val endPose = Pose2d()
    }

    fun m() {
        while (opModeIsActive()) {
            arm.update()
            drive.update()

            if (!drive.isBusy) {
                claw.open()
                when (dir) {
                    AprilTagPipeline.Tag.LEFT  -> drive.followTrajectory(
                        drive.trajectoryBuilder(LeftTraj.endPose)
                            .forward(28.0)
                            .build()
                    )
                    AprilTagPipeline.Tag.RIGHT -> drive.followTrajectory(
                        drive.trajectoryBuilder(LeftTraj.endPose)
                            .back(26.0)
                            .build()
                    )
                    else                       -> {}
                }
                break
            }
        }
    }
}

private fun Claw.open() {


}
