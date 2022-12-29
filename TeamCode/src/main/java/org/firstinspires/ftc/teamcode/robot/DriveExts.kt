package org.firstinspires.ftc.teamcode.robot


import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin
import kotlin.reflect.KProperty

/**
 * Updates powers to field centric values based on gamepad input, imu angle, and [speed].
 * This depends on a modification to the SampleMecanumDrive class, making the 'imu' field public.
 *
 * @see SampleMecanumDrive.imu
 */
@Deprecated("if the new DriveExt class doesn't blow up this will be removed")
fun SampleMecanumDrive.fieldcentric(gp1: GamepadExt, gp2: GamepadExt) {
    val y = -gp1.left_stick_y.toDouble()
    val x = gp1.left_stick_x * 1.1
    val turn = gp1.right_stick_x.toDouble()

    val botHeading = rawExternalHeading
    val rotX = x * cos(botHeading) - y * sin(botHeading)
    val rotY = x * sin(botHeading) + y * cos(botHeading)

    val denom = max(abs(y) + abs(x) + abs(turn), 1.0)
    setMotorPowers(
        (rotY + rotX + turn) / denom * speed,
        (rotY - rotX + turn) / denom * speed,
        (rotY + rotX - turn) / denom * speed,
        (rotY - rotX - turn) / denom * speed,
    )
    update()
}

/**
 * Updates powers to robot centric values based on gamepad input and [speed].
 */
@Deprecated("")
fun SampleMecanumDrive.robotcentric(gp1: GamepadExt, gp2: GamepadExt) {
    setWeightedDrivePower(
        Pose2d(
            -gp1.left_stick_y.toDouble() * speed, -gp1.left_stick_x.toDouble() * speed,
            -gp1.right_stick_x.toDouble() * speed
        )
    )
    update()
}

/**
 * @see SampleMecanumDrive.update(gp1: GamepadExt, gp2: GamepadExt)
 */
@Deprecated(
    "", ReplaceWith(
        "DriveExt.update(Pair<GamepadExt, GamepadExt>)"
    )
)
fun SampleMecanumDrive.update(gps: Pair<GamepadExt, GamepadExt>, robotCentric: Boolean = false) {
    if (!robotCentric) fieldcentric(gps.first, gps.second)
    else robotcentric(gps.first, gps.second)
}

/**
 * Constant that powers will be multiplied by in extension functions that set motor powers. This
 * value is static to the SampleMecanumDrive class.
 *
 * @see SampleMecanumDrive.robotcentric
 * @see SampleMecanumDrive.update
 */
@Deprecated("", ReplaceWith("DriveExt.speed")) var SampleMecanumDrive.speed: Double by object {
    private var backingField = 0.6
    operator fun getValue(thisRef: SampleMecanumDrive, property: KProperty<*>) = backingField
    operator fun setValue(thisRef: SampleMecanumDrive, property: KProperty<*>, value: Double) {
        backingField = value
    }
}
