package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.PIDController
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.vision.*
import org.openftc.easyopencv.OpenCvWebcam

@TeleOp(group = "a")
class RedTeleop : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm3
    private lateinit var drive: DriveExt
    private lateinit var gamepads: Pair<GamepadExt, GamepadExt>
    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    private lateinit var frontWebcam: OpenCvWebcam
    private lateinit var backWebcam: OpenCvWebcam
    private val coneDetector = ConeDetectionPipeline.redConeDetector()
    private val poleDetector = PoleDetectionPipeline()
    private val conePID = PIDController(0.001, 0.0, 0.0)
    private val polePID = PIDController(0.001, 0.0, 0.0)
    override fun runOpMode() {
        gamepads = GamepadExt(gamepad1) to GamepadExt(gamepad2)
        arm = Arm3(hardwareMap)
        claw = Claw(hardwareMap)
        drive = DriveExt(hardwareMap).apply { poseEstimate = DriveExt.PoseStorage.pose }

        backWebcam = createWebcam(
            hardwareMap, RobotConfig.WEBCAM_2, pipeline = poleDetector,
        )
        frontWebcam = createWebcam(
            hardwareMap, RobotConfig.WEBCAM_1, pipeline = coneDetector
        )

        val gp1 = gamepads.first
        val gp2 = gamepads.second

        EventLoop(::opModeIsActive, tm).apply {
            onPressed(gp1::dpad_up, gp2::dpad_up) { arm.state = Arm.next(arm.state) }
            onPressed(gp1::dpad_down, gp2::dpad_down) { arm.state = Arm.prev(arm.state) }

            onPressed(gp2::a) { arm.state = Arm.GROUND }
            onPressed(gp2::x) { arm.state = Arm.MID }
            onPressed(gp2::y) { arm.state = Arm.HIGH }
            onPressed(gp2::b) { arm.state = Arm.BACKHIGH }

            onPressed(gp2::left_bumper) { claw.change() }

            /* sprint mode */
            onMoved(gp1::left_trigger) { drive.state = DriveExt.SPRINTING }
            onReturned(gp1::left_trigger) { drive.state = DriveExt.NORMAL }

            /* aim at cone */
            var coneAiming = false
            onPressed(gp1::right_bumper, gp2::right_bumper) {
                coneAiming = !coneAiming
                if (coneAiming) {
                    claw.state = Claw.OPENED
                }
            }
            runIf({ coneAiming }) {
                if (coneDetector.detected) gp1.right_stick_x =
                    conePID.calculate(coneDetector.error).toFloat().also { println("cone aim $it") }

                if (claw.state == Claw.CLOSED) {
                    coneAiming = false
                }
            }

            /* aim at pole */
            var poleAiming = false
            onMoved(gp1::right_trigger, gp2::right_trigger) {
                poleAiming = !poleAiming
            }
            runIf({ poleAiming }) {
                if (poleDetector.detected) gp1.right_stick_x =
                    -polePID.calculate(poleDetector.error)
                        .toFloat()
                        .also { println("pole aim $it") }

                if (claw.state != Claw.CLOSED) {
                    poleAiming = false
                }
            }

            var cycling = false
            onPressed(gp1::dpad_left) {
                cycling = !cycling
                if (arm.state != Arm.GROUND || claw.state != Claw.CLOSED) cycling = false
                if (cycling) {
                    drive.followTrajectoryAsync(
                        drive.trajectoryBuilder(Pose2d()).back(32.0).build()
                    )
                    arm.state = Arm.BACKHIGH
                    poleAiming = true
                } else drive.exitTrajectory()
            }
            runIf({ cycling }) { if (!drive.isBusy) cycling = false }

            updates += listOf({ drive.update(gamepads) }, { arm.update() }, { tm.update(); Unit }, {
                if (drive.state != DriveExt.SPRINTING) {
                    if (arm.state > Arm.STACK && drive.state == DriveExt.NORMAL) drive.state =
                        DriveExt.SLOW
                    if (arm.state <= Arm.STACK && drive.state == DriveExt.SLOW) drive.state =
                        DriveExt.NORMAL
                }
            }, { gamepads.sync() })
        }.also {
            waitForStart()
            it.run()
        }
    }
}
