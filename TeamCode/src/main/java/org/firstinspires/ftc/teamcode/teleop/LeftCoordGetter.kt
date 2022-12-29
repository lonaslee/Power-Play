package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.Arm
import org.firstinspires.ftc.teamcode.robot.Claw
import org.firstinspires.ftc.teamcode.robot.GamepadExt
import org.firstinspires.ftc.teamcode.robot.sync
import org.firstinspires.ftc.teamcode.robot.update

@TeleOp
class LeftCoordGetter : OpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var drive: SampleMecanumDrive
    private lateinit var gamepads: Pair<GamepadExt, GamepadExt>

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun init() {
        gamepads = GamepadExt(gamepad1) to GamepadExt(gamepad2)
        arm = Arm(hardwareMap)
        claw = Claw(hardwareMap)
        drive = SampleMecanumDrive(hardwareMap)/* .apply { poseEstimate = Trajectories.leftStartPos } */
    }

    override fun loop() {
        drive.setWeightedDrivePower(
            Pose2d(
                (-gamepad1.left_stick_y).toDouble() * .4, (-gamepad1.left_stick_x).toDouble() * .4,
                (-gamepad1.right_stick_x).toDouble() * .4
            )
        )
        drive.update()

        val (x, y, heading) = drive.poseEstimate
        telemetry.addData("x", x)
        telemetry.addData("y", y)
        telemetry.addData("heading", Math.toDegrees(heading))

        gamepads.sync()
        tm.update()
    }
}
