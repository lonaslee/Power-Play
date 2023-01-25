package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.PIDController
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.vision.*
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam

@TeleOp
@com.acmerobotics.dashboard.config.Config
class TeleOp3 : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm2
    private lateinit var drive: DriveExt
    private lateinit var gamepads: Pair<GamepadExt, GamepadExt>
    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    private lateinit var frontWebcam: OpenCvWebcam
    private lateinit var backWebcam: OpenCvWebcam
    private val coneDetector = ConeDetectionPipeline.redConeDetector()
    private val poleDetector = PoleDetectionPipeline(tm)
    private val conePID = PIDController(pP, pI, pD)
    private val polePID = PIDController(jP, jI, jD)

    override fun runOpMode() {
        gamepads = GamepadExt(gamepad1) to GamepadExt(gamepad2)
        arm = Arm2(hardwareMap, tm)
        claw = Claw(hardwareMap)
        drive = DriveExt(hardwareMap).apply { poseEstimate = DriveExt.PoseStorage.pose }

        frontWebcam = createWebcam(
            hardwareMap, RobotConfig.WEBCAM_1, pipeline = coneDetector
        ).apply { stopStreaming() }
        backWebcam = createWebcam(
            hardwareMap, RobotConfig.WEBCAM_2, pipeline = poleDetector,
        ).apply { stopStreaming() }

        val gp1 = gamepads.first
        val gp2 = gamepads.second

        EventLoop(::opModeIsActive, tm).apply {
            onPressed(gp1::dpad_up) { arm.state = Arm.next(arm.state) }
            onPressed(gp1::dpad_down) { arm.state = Arm.prev(arm.state) }

            onPressed(gp1::a, gp2::a) { arm.state = Arm.GROUND }
            onPressed(gp1::x, gp2::x) { arm.state = Arm.MID }
            onPressed(gp1::y, gp2::y) { arm.state = Arm.HIGH }
            onPressed(gp1::b, gp2::b) { arm.state = Arm.BACKHIGH }

            onPressed(gp1::left_bumper, gp2::left_bumper) { claw.change() }

            /* sprint mode */
            onMoved(gp1::right_trigger) { drive.speed = 1.0 }
            onReturned(gp1::right_trigger) { drive.speed = 0.7 }

            /* aim at cone */
            var coneAiming = false
            onPressed(gp1::right_bumper, gp2::right_bumper) {
                coneAiming = !coneAiming
                if (coneAiming) {
                    claw.state = Claw.OPENED
                    frontWebcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT)
                } else frontWebcam.stopStreaming()
            }
            runIf({ coneAiming }) {
                if (coneDetector.detected) gp1.right_stick_x =
                    conePID.calculate(coneDetector.error).toFloat().also { println("cone aim $it") }

                if (claw.state == Claw.CLOSED) {
                    coneAiming = false
                    frontWebcam.stopStreaming()
                }
            }

            /* aim at pole */
            var poleAiming = false
            onMoved(gp1::left_trigger, gp2::left_trigger) {
                poleAiming = !poleAiming
                if (poleAiming) backWebcam.startStreaming(
                    CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN
                ) else backWebcam.stopStreaming()
            }
            runIf({ poleAiming }) {
                if (poleDetector.detected) gp1.right_stick_x =
                    polePID.calculate(poleDetector.error).toFloat().also { println("pole aim $it") }

                if (claw.state != Claw.CLOSED) {
                    poleAiming = false
                    backWebcam.stopStreaming()
                }
            }

            /* cycle a cone
            var cycling = false
            onPressed(gp1::back) {
                cycling = !cycling
                if (arm.state != Arm.GROUND || claw.state != Claw.CLOSED) cycling = false
                if (cycling) {
                    drive.followTrajectorySequenceAsync(
                        drive.trajectorySequenceBuilder(drive.poseEstimate)
                            .setReversed(true)
                            .splineTo(Vector2d(0.0, -32.0), 90.rad)
                            .build()
                    )
                    arm.state = Arm.BACKHIGH
                    poleAiming = true
                } else drive.exitTrajectory()
            }
            runIf({ cycling }) { if (!drive.isBusy) cycling = false }
*/
            updates += listOf({ drive.update(gamepads) },
                { arm.update() },
                { tm.update(); Unit },
                { gamepads.sync() },
                { conePID.constants = Triple(pP, pI, pD) },
                { polePID.constants = Triple(jP, jI, jD) })
        }.also {
            waitForStart()
            it.run()
        }
    }

    companion object {
        @JvmField var jP = 0.001
        @JvmField var jI = 0.0
        @JvmField var jD = 0.0

        @JvmField var pP = 0.001
        @JvmField var pI = 0.0
        @JvmField var pD = 0.0
    }
}
