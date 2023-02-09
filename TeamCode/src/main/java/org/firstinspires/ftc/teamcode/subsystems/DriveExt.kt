package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import kotlin.math.*

class DriveExt(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry? = null,
) : SampleMecanumDrive(hardwareMap), Subsystem {
    override var state = NORMAL

    fun update(gamepads: Pair<Gamepad, Gamepad>) {
        if (!isBusy) driveFieldCentric(gamepads)
        super.update()
    }

    /**
     * Updates powers to field centric values based on gamepad input, imu angle, and [state].
     */
    private fun driveFieldCentric(gamepads: Pair<Gamepad, Gamepad>) {
        val y = -gamepads.first.left_stick_y.toDouble()
        val x = gamepads.first.left_stick_x * 1.1
        val turn = gamepads.first.right_stick_x.toDouble()

        val (rotX, rotY) = (-rawExternalHeading + PI).let {
            Pair(x * cos(it) - y * sin(it), x * sin(it) + y * cos(it))
        }

        val denom = max(abs(y) + abs(x) + abs(turn), 1.0)
        setMotorPowers(
            (rotY + rotX + turn) / denom * state,
            (rotY - rotX + turn) / denom * state,
            (rotY + rotX - turn) / denom * state,
            (rotY - rotX - turn) / denom * state,
        )
    }

    /**
     * Stop following the current trajectory, if there is one.
     */
    fun exitTrajectory() {
        trajectorySequenceRunner.currentTrajectorySequence = null
        trajectorySequenceRunner.remainingMarkers.clear()
    }

    class OdoRetract(hardwareMap: HardwareMap) : Subsystem {
        private val servo = (hardwareMap[RobotConfig.RETRACTOR.s] as ServoImplEx).apply {
            direction = Servo.Direction.FORWARD
            position = DOWN
        }

        override var state = DOWN
            set(value) {
                if (field == value) return
                field = value
                servo.position = value
            }

        companion object : Subsystem.States {
            const val RETRACTED = 0.72
            const val DOWN = 0.45

            override val all = listOf(RETRACTED, DOWN)
        }
    }

    val midodo = OdoRetract(hardwareMap)

    companion object States : Subsystem.States {
        const val SPRINTING = 1.0
        const val NORMAL = 0.7
        const val SLOW = 0.5

        override val all = listOf(SLOW, NORMAL, SPRINTING)
    }
}
