package org.firstinspires.ftc.teamcode.constants.exceptions;

import androidx.annotation.NonNull;

public class NoAssociatedConstantsFileException extends Exception {
    @NonNull public final String fileName;

    public NoAssociatedConstantsFileException(@NonNull String fileName) {
       this.fileName = fileName;
    }
}
