package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.GamepadExt
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

class DriveExt(
    hardwareMap: HardwareMap, private val telemetry: Telemetry? = null, private var robotcentric: Boolean = false
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
        if (!robotcentric) fieldcentric(gamepads)
        else robotcentric(gamepads)
        super.update()
        lastPose = poseEstimate
    }

    var speed = 0.6

    /**
     * Updates powers to field centric values based on gamepad input, imu angle, and [speed].
     */
    private fun fieldcentric(gamepads: Pair<GamepadExt, GamepadExt>) {
        val y = -gamepads.first.left_stick_y.toDouble()
        val x = gamepads.first.left_stick_x * 1.1
        val turn = gamepads.first.right_stick_x.toDouble()

        val (rotX, rotY) = (-rawExternalHeading).let { Pair(x * cos(it) - y * sin(it), x * sin(it) + y * cos(it)) }

        val denom = max(abs(y) + abs(x) + abs(turn), 1.0)
        setMotorPowers(
            (rotY + rotX + turn) / denom * speed,
            (rotY - rotX + turn) / denom * speed,
            (rotY + rotX - turn) / denom * speed,
            (rotY - rotX - turn) / denom * speed,
        )
    }

    /**
     * Updates powers to robot centric values based on gamepad input and [speed].
     */
    private fun robotcentric(gp1: Pair<GamepadExt, GamepadExt>) {
        setWeightedDrivePower(
            Pose2d(
                -gp1.first.left_stick_y.toDouble() * speed, -gp1.first.left_stick_x.toDouble() * speed,
                -gp1.first.right_stick_x.toDouble() * speed
            )
        )
    }
}
