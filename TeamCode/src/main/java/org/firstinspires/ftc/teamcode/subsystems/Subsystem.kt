package org.firstinspires.ftc.teamcode.subsystems


interface Subsystem {
    interface States {
        val all: List<Number>

        fun next(this_: Number) = if (this_ == all.last()) this_ else all[all.indexOf(this_) + 1]
        fun prev(this_: Number) = if (this_ == all.first()) this_ else all[all.indexOf(this_) - 1]
    }

    val state: Number
}
