package org.firstinspires.ftc.teamcode.vision

import org.openftc.easyopencv.OpenCvPipeline

abstract class SignalSleevePipeline : OpenCvPipeline() {
    abstract val verdict: Tag

    enum class Tag {
        LEFT, MIDDLE, RIGHT, UNKNOWN
    }
}
