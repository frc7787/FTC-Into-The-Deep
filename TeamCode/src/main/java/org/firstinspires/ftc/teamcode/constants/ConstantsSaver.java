package org.firstinspires.ftc.teamcode.constants;

import static org.firstinspires.ftc.teamcode.constants.Constants.FileConstants.*;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Scalar;

import java.io.*;
import java.lang.reflect.*;
import java.util.*;

/**
 * <h1>ConstantsSaver</h1>
 * <p>
 *     The ConstantsSaver class attempts to save all of the values found in {@link Constants} nested
 *     classes to their corresponding text file found in onbot java. If the corresponding file is
 *     not already present, then one will be created. The values are save in the following location:
 * </p>
 * <p>
 *     /sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/Constants/
 * </p>
 * <p>
 *     When the constants loader is saving a class to its corresponding text file, it iterates
 *     through each field and converts it to a string which is typically in the form:
 * </p>
 * <p>
 *     FIELD_NAME=VALUE
 * </p>
 * <p>
 *     One exception to this rule is Pose2D which is saved as:
 * </p>
 * <p>
 *     FIELD_NAME=Pose2D(X,Y,HEADING)
 * </p>
 * <p>
 *     When a field is encountered that cannot be saved, it is silently ignored. Any of the
 *     following will cause a field to be skipped:
 *     <ul>
 *         <li>The field is private, not static, or final</li>
 *         <li>The field type is not supported. See {@link SupportedType}</li>
 *     </ul>
 * </p>
 * <p>
 *     By default all failures to save a field are ignored silently. However, in some instances
 *     displaying the reason can be useful if you are debugging. To enable this feature, call the
 *     constructor with the opModes telemetry object.
 * </p>
 */
public final class ConstantsSaver {
    private final boolean debug;
    private final Telemetry telemetry;
    private final ArrayList<String> debugOutput;

    /**
     * Creates a new ConstantsSaver class with debugging disabled
     */
    public ConstantsSaver() {
        debug       = false;
        telemetry   = null;
        debugOutput = new ArrayList<>();
    }

    /**
     * Creates a new ConstantsSaver class with debugging enabled.
     * @param telemetry The telemetry to display debug information
     */
    public ConstantsSaver(@NonNull Telemetry telemetry) {
        this.telemetry = telemetry;
        debug          = true;
        debugOutput    = new ArrayList<>();
    }

    /**
     * <p>
     *     Attempts to save all of the values located in {@link Constants} to their corresponding
     *     text files located at:
     * </p>
     * <p>
     *     /sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/Constants
     * </p>
     * <p>
     *     If their is no corresponding text file, one will be created.
     * </p>
     * <p>
     *     By default all errors are silently ignored. To enable debug mode, pass the OpModes
     *     telemetry object into the constructor of the ConstantsLoader class.
     * </p>
     */
    public void save() {
        for (Class<?> clazz : Constants.class.getDeclaredClasses()) {
            if (!Modifier.isStatic(clazz.getModifiers())) {
                String issue = "Skipped Class " + clazz.getSimpleName()
                             + "\nReason: Class Is Not Static";
                debugOutput.add(issue);
                continue;
            }
            saveClass(clazz);
        }

        if (!debug) return;

        telemetry.addData("Number Of Issues Found", debugOutput.size());
        for (String output : debugOutput) { telemetry.addLine("\n" + output); }
    }

    private void saveClass(@NonNull Class<?> clazz) {
        String fileName = CONSTANTS_FILE_LOCATION + clazz.getSimpleName() + ".txt";

        File constantsFile = new File(fileName);

        try (BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(constantsFile))) {
            Field[] fields = clazz.getFields();

            if (fields.length == 0) {
                String issue = "Skipped Class " + clazz.getSimpleName()
                             + "\nReason: No Accessible Fields";
                debugOutput.add(issue);
                return;
            }

            for (Field field : fields) { saveField(bufferedWriter, field); }

            bufferedWriter.flush();
        } catch (IOException ioException) {
            String issue = "Failed To Create File " + fileName
                         + "\nReason: " + ioException.getMessage();
            debugOutput.add(issue);
        }
    }

    private void saveField(@NonNull BufferedWriter bufferedWriter, @NonNull Field field) {
        if (!isSavable(field)) return;

        String line;
        String fieldName = field.getName();

        try {
            switch (SupportedType.fieldToType(field)) {
                case SPARK_FUN_POSE_2D:
                    line = fieldName + "=" + pose2DString((SparkFunOTOS.Pose2D) field.get(null));
                    break;
                case SCALAR:
                    line = fieldName + "=" + scalarString((Scalar) field.get(null));
                    break;
                case UNSUPPORTED:
                    String issue = "Failed To Save Field " + fieldName
                                 + "\nReason: " + field.getType().getSimpleName() + " Is Not A"
                                 + "\nSupported Type.";
                    debugOutput.add(issue);
                    return;
                default: // All other types don't need anything special to save them
                    line = fieldName + "=" + field.get(null);
                    break;
            }
        } catch (IllegalAccessException | IllegalArgumentException ignored) { return; }

        try {
            bufferedWriter.write(line);
            bufferedWriter.newLine();
        } catch (IOException ioException) {
            String issue = "Failed To Save Field " + field.getName() + "\n"
                         + "Reason: " + ioException.getMessage();
            debugOutput.add(issue);
        }
    }

    @NonNull private String pose2DString(@NonNull SparkFunOTOS.Pose2D pose2D) {
        return "Pose2D(" + pose2D.x + "," + pose2D.y + "," + pose2D.h + ")";
    }

    @NonNull private String scalarString(@NonNull Scalar scalar) {
        return "Scalar(" + scalar.val[0] + "," + scalar.val[1] + "," + scalar.val[3] + ")";
    }

    private boolean isSavable(@NonNull Field field) {
        int modifiers = field.getModifiers();

        boolean isPublic = Modifier.isPublic(modifiers);
        boolean isFinal  = Modifier.isFinal(modifiers);
        boolean isStatic = Modifier.isStatic(modifiers);

        if (!isPublic || isFinal || !isStatic) {
            String issue = "Failed To Save Field" + field.getName();

            if (!isPublic) issue += "Reason: Field Is Not Public";
            if (!isFinal)  issue += "Reason: Field Is Final";
            if (!isStatic) issue += "Reason: Field Is Not Static";

            debugOutput.add(issue);
            return false;
        }
        return true;
    }
}
