package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.teleop.GamepadExt
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

class DriveExt(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry? = null,
) : SampleMecanumDrive(hardwareMap), Subsystem {
    private var lastPose = Pose2d()

    companion object States : Subsystem.States {
        const val STATIONARY = 0
        const val MOVING = 1

        override val all = listOf(0, 1)
    }

    override val state
        get() = if (lastPose != poseEstimate) MOVING else STATIONARY

    fun update(gamepads: Pair<GamepadExt, GamepadExt>) {
        if (!isBusy) {
            driveFieldCentric(gamepads)
            lastPose = poseEstimate
        }
        super.update()
    }

    var speed = 0.8

    /**
     * Updates powers to field centric values based on gamepad input, imu angle, and [speed].
     */
    private fun driveFieldCentric(gamepads: Pair<GamepadExt, GamepadExt>) {
        val y = -gamepads.first.left_stick_y.toDouble()
        val x = gamepads.first.left_stick_x * 1.1
        val turn = gamepads.first.right_stick_x.toDouble()

        val (rotX, rotY) = (rawExternalHeading).let {
            Pair(x * cos(it) - y * sin(it), x * sin(it) + y * cos(it))
        }

        val denom = max(abs(y) + abs(x) + abs(turn), 1.0)
        setMotorPowers(
            (rotY + rotX + turn) / denom * speed,
            (rotY - rotX + turn) / denom * speed,
            (rotY + rotX - turn) / denom * speed,
            (rotY - rotX - turn) / denom * speed,
        )
    }

    /**
     * Stop following the current trajectory, if there is one.
     */
    fun exitTrajectory() {
        trajectorySequenceRunner.currentTrajectorySequence = null
        trajectorySequenceRunner.remainingMarkers.clear()
    }

    object PoseStorage {
        /**
         * Stores the pose of the robot at the end of autonomous.
         */
        var pose = Pose2d()
    }
}
