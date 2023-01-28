package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.PIDController
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.HIGH
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.GROUND
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.LOW
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.LOWER
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.MID
import org.firstinspires.ftc.teamcode.subsystems.Arm.States.STACK
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Claw.States.CLOSED
import org.firstinspires.ftc.teamcode.subsystems.Claw.States.OPENED
import org.firstinspires.ftc.teamcode.subsystems.DriveExt
import org.firstinspires.ftc.teamcode.subsystems.RobotConfig
import org.firstinspires.ftc.teamcode.vision.ConeDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.createWebcam
import org.openftc.easyopencv.OpenCvWebcam
import kotlin.math.PI
import kotlin.math.abs

@TeleOp(group = "test")
@com.acmerobotics.dashboard.config.Config
class TeleOp4 : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var drive: DriveExt
    private lateinit var webcam: OpenCvWebcam
    private val pipeline = ConeDetectionPipeline.redConeDetector()
    private val pickPID = PIDController(pP, pI, pD)
    private val turnPID = PIDController(tP, tI, tD)

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun runOpMode() {
        arm = Arm(hardwareMap)
        claw = Claw(hardwareMap)
        drive = DriveExt(hardwareMap)
        webcam = createWebcam(hardwareMap, RobotConfig.WEBCAM_1)

        val gp1 = GamepadExt(gamepad1)
        val gp2 = GamepadExt(gamepad2)
        val gamepads = gp1 to gp2

        EventLoop(::opModeIsActive, tm).apply {
            updates += listOf({ arm.update() },
                { drive.update(gamepads) },
                { gamepads.sync() },
                { tm.update(); Unit },
                { pickPID.constants = Triple(pP, pI, pD) },
                { turnPID.constants = Triple(tP, tI, tD) })

            onPressed(gp1::dpad_up, gp2::dpad_up) { arm.state = Arm.next(arm.state) }
            onPressed(gp1::dpad_down, gp2::dpad_down) { arm.state = Arm.prev(arm.state) }
            onPressed(gp1::a, gp2::a) { arm.state = GROUND }
            onPressed(gp1::x, gp2::x) { arm.state = LOW }
            onPressed(gp1::y, gp2::y) { arm.state = MID }
            onPressed(gp1::b, gp2::b) { arm.state = HIGH }

            onPressed(gp1::left_bumper, gp2::left_bumper) {
                when (arm.state) {
                    GROUND, STACK -> claw.change()
                    in Arm.all -> arm.state = LOWER
                    else -> claw.state = OPENED // arm is lowered
                }
            }

            /* aim at cone */
            var aiming = false
            onPressed(gp1::right_bumper, gp2::right_bumper) {
                aiming = (!aiming)
                if (aiming) claw.state = OPENED
            }
            runIf({ aiming }) {
                if (pipeline.detected) gp1.right_stick_x =
                    pickPID.calculate(pipeline.error).toFloat().also { println("aim $it") }
                if (claw.state == CLOSED) aiming = false
            }


            fun wrap(ang: Double) =
                if (ang > PI) ang - 2 * PI else if (ang < -PI) ang + 2 * PI else ang

            /* turn 180 left */
            var leftAngle: Double? = null

            onPressed(gp1::dpad_left) {
                leftAngle = if (leftAngle == null) wrap(drive.poseEstimate.heading + PI)
                else null
                println("left target : $leftAngle")
            }

            runIf({
                leftAngle != null && (gp1.right_stick_x == 0F && wrap(drive.poseEstimate.heading) !in leftAngle!! - 2.rad..leftAngle!! + 2.rad).also {
                    if (!it) leftAngle = null
                }
            }) {
                gp1.right_stick_x =
                    -abs(turnPID.calculate(leftAngle!!, wrap(drive.poseEstimate.heading))).toFloat()
                        .also { println("left : ${wrap(drive.poseEstimate.heading)} $it") }
            }

            /* turn 180 right */
            var rightAngle: Double? = null

            onPressed(gp1::dpad_right) {
                rightAngle = if (rightAngle == null) wrap(drive.poseEstimate.heading - PI)
                else null
                println("right target : $rightAngle")
            }

            runIf({
                rightAngle != null && (gp1.right_stick_x == 0F && wrap(drive.poseEstimate.heading) !in (rightAngle!! - 2.rad)..(rightAngle!! + 2.rad)).also {
                    if (!it) rightAngle = null
                }
            }) {
                gp1.left_stick_x =
                    abs(turnPID.calculate(rightAngle!!, wrap(drive.poseEstimate.heading))).toFloat()
                        .also { println("right : ${wrap(drive.poseEstimate.heading)} - $it") }
            }
        }.also {
            waitForStart()
            it.run()
        }
    }

    companion object {
        @JvmField var pP = 0.002
        @JvmField var pI = 0.0
        @JvmField var pD = 0.0

        @JvmField var tP = 0.01
        @JvmField var tI = 0.0
        @JvmField var tD = 0.0

        private val Int.rad get() = this * PI / 180
    }
}
