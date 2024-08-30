package org.firstinspires.ftc.teamcode.constants;

import static org.firstinspires.ftc.teamcode.constants.Constants.FileConstants.*;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.*;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Scalar;

import java.io.*;
import java.lang.reflect.*;
import java.util.*;
import java.util.stream.*;

/**
 * <h1>Constants Loader</h1>
 * <p>
 *     The constants loader class attempts to override the values located in each static nested
 *     class found within the {@link Constants} using values found in text files corresponding names
 *     located at the following path:
 * </p>
 * <p>
 *     /sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/Constants/
 * </p>
 * <p>
 *     If a static nested class doesn't have a corresponding text file, it will be silently ignored.
 * </p>
 * <p>
 *     When the constants loader locates a file that corresponds with one of the nested constants
 *     classes, it will iterate through each field of the class and attempt to find a matching field
 *     in the file. In order to be recognized by the ConstantsLoader the field in the file must be
 *     formatted as follows:
 * </p>
 * <p>
 *     FIELD_NAME=VALUE
 * </p>
 * <p>
 *     One exception to this rule is Pose2D which must be formatted as follows:
 * </p>
 * <p>
 *     FIELD_NAME=Pose2D(X,Y,HEADING)
 * </p>
 * <p>
 *     The flavour of Pose2D (Roadrunner, SparkFunOTOS, etc) doesn't matter as the value in the text
 *     file will be assigned to the type of the field in the nested class.
 * </p>
 * <p>
 *     Any of the following conditions will cause the ConstantsLoader to skip overwriting a field:
 *     <ul>
 *         <li>The class field has no corresponding file field and vice-versa.</li>
 *         <li>The file field is not formatted as shown above.</li>
 *         <li>The file field and class field types mismatch.</li>
 *         <li>The class field is not static, not public, or final.</li>
 *     </ul>
 * </p>
 * <p>
 *     By default all failures to overwrite fields in the nested constants classes are ignored
 *     silently. However, in some instances displaying the reason can be useful if you are
 *     debugging. To enable this feature, call the constructor with the opModes telemetry object.
 * </p>
 */
public final class ConstantsLoader {
    private final boolean debug;

    @NonNull private final Debugger debugger;

    /**
     * Creates a new ConstantsLoader with debugging disabled. To enable debugging, pass the opModes
     * telemetry object to the constructor.
     */
    public ConstantsLoader() {
        this.debugger = new Debugger(null);
        this.debug    = false;
        MalformedPropertyException.debug = false;
    }

    /**
     * Creates a new constants loader with debugging enabled.
     * @param telemetry The telemetry to display debug information on
     */
    public ConstantsLoader(@NonNull Telemetry telemetry) {
        this.debug    = true;
        this.debugger = new Debugger(telemetry);
        MalformedPropertyException.debug = true;
    }


    @NonNull private List<File> getConstantsFilesFromOnbotJava() {
        File constantsDirectory = new File(CONSTANTS_FILE_LOCATION);

        if (constantsDirectory.isFile()) {
            if (!debug) return new ArrayList<>();

            String issue = "Failed To Load Constants File Directory"
                         + "\nReason: Conflicting File Constants in OnBot Java Directory."
                         + "\nHelp: Remove Directory Named \"Constants\"";

            debugger.addMessage(issue);

            return new ArrayList<>();
        }

        File[] constantsDirectoryFiles = constantsDirectory.listFiles();

        if (constantsDirectoryFiles == null) {
            if (!debug) return new ArrayList<>();

            String issue = "Failed To Load Constants Directory"
                         + "\nReason: No Files Were Found At The Specified Constants Directory";
            debugger.addMessage(issue);
            return new ArrayList<>();
        }

        List<File> textFiles = Arrays.stream(constantsDirectoryFiles)
                .filter(this::isTextFile)
                .collect(Collectors.toList());

        if (textFiles.isEmpty()) {
            String issue = "Failed To Load Constants Directory"
                         + "\nReason: No Files With Extension \".txt\" were found.";
            if (debug) debugger.addMessage(issue);
        }
        return textFiles;
    }

    private @NonNull Optional<Class<?>> matchConstantsClassToConstantsFile(
            @NonNull String fileName
    ) {
        for (Class<?> clazz : Constants.class.getClasses()) {
            if (!Modifier.isStatic(clazz.getModifiers())) continue;

            if (clazz.getSimpleName().equals(fileName)) return Optional.of(clazz);
        }
        String issue = "Failed To Match Constants Class With Name: " + fileName
                     + "\nNote: Non-Static Nested Classes Are Skipped";
        if (debug) debugger.addMessage(issue);
        return Optional.empty();
    }

    private void populateClassFromPropertiesFile(
            @NonNull Class<?> clazz,
            @NonNull String fileName
    ) {
        Properties properties = new Properties();

        try {
            properties.load(new FileInputStream(CONSTANTS_FILE_LOCATION + fileName));
            for (Field field : clazz.getFields()) { populateField(field, properties); }
        } catch (IOException ioException) {
            String issue = "Failed To Load Constants File " + fileName
                         + "\nReason: " + ioException.getMessage();
            if (debug) debugger.addMessage(issue);
        }
    }

    private void populateField(@NonNull Field field, @NonNull Properties properties) {
        String fieldName = field.getName();

        if (!properties.containsKey(fieldName) || !isLoadable(field)) return;

        try {
            switch (SupportedType.fieldToType(field)) {
                case FLOAT:
                    field.setFloat(fieldName, loadFloat(fieldName, properties));
                    break;
                case DOUBLE:
                    field.setDouble(fieldName, loadDouble(fieldName, properties));
                    break;
                case BYTE:
                    field.setByte(fieldName, loadByte(fieldName, properties));
                    break;
                case SHORT:
                    field.setShort(fieldName, loadShort(fieldName, properties));
                    break;
                case INTEGER:
                    field.setInt(fieldName, loadInteger(fieldName, properties));
                    break;
                case LONG:
                    field.setLong(fieldName, loadLong(fieldName, properties));
                    break;
                case BOOLEAN:
                    field.setBoolean(field, loadBoolean(fieldName, properties));
                    break;
                case CHAR:
                    field.setChar(field, loadCharacter(fieldName, properties));
                case STRING:
                    field.set(fieldName, loadString(fieldName, properties));
                    break;
                case SERVO_DIRECTION:
                    try {
                        Servo.Direction servoDirection = loadServoDirection(fieldName, properties);
                        field.set(fieldName, servoDirection);
                    } catch (MalformedPropertyException ignored) { return; }
                    break;
                case MOTOR_DIRECTION:
                    try {
                        DcMotorSimple.Direction motorDirection
                                = loadMotorDirection(fieldName, properties);
                        field.set(fieldName, motorDirection);
                    } catch (MalformedPropertyException ignored) { return; }
                    break;
                case ZERO_POWER_BEHAVIOUR:
                    try {
                        DcMotor.ZeroPowerBehavior zeroPowerBehavior
                                = loadZeroPowerBehaviour(fieldName, properties);
                        field.set(fieldName, zeroPowerBehavior);
                    } catch (MalformedPropertyException ignored) { return; }
                    break;
                case SPARK_FUN_POSE_2D:
                    try {
                        SparkFunOTOS.Pose2D pose2D
                                = loadSparkFunPose2D(fieldName, properties);
                        field.set(fieldName, pose2D);
                    } catch (MalformedPropertyException ignored) { return; }
                    break;
                case RUN_MODE:
                    try {
                        DcMotor.RunMode runMode
                                = loadRunMode(fieldName, properties);
                        field.set(fieldName, runMode);
                    } catch (MalformedPropertyException ignored) { return; }
                    break;
                case SCALAR:
                    try {
                        Scalar scalar
                                = loadScalar(fieldName, properties);
                        field.set(fieldName, scalar);
                    } catch (MalformedPropertyException ignored) { return; }
                    break;
                case LOGO_FACING_DIRECTION:
                    try {
                        LogoFacingDirection logoFacingDirection
                                = loadLogoFacingDirection(fieldName, properties);
                        field.set(fieldName, logoFacingDirection);
                    } catch (MalformedPropertyException ignored) { return; }
                    break;
                case USB_FACING_DIRECTION:
                    try {
                        UsbFacingDirection usbFacingDirection
                            = loadUsbFacingDirection(fieldName, properties);
                        field.set(fieldName, usbFacingDirection);
                    } catch (MalformedPropertyException ignored) { return; }
                    break;
                case UNSUPPORTED:
                     if (debug) {
                         String message = "Failed To Load Field " + fieldName
                                 + "\nReason: Field Type Not Supported";
                         debugger.addMessage(message);
                     }
                break;
            }
        } catch (IllegalAccessException ignored) {}
    }

    /**
     * <p>
     *  Attempts to overwrite all of the values located in {@link Constants} with values found in
     *  corresponding text files located at the following path:
     * </p>
     * <p>
     *     /sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/Constants
     * </p>
     * <p>
     *     By default all errors are silently ignored. To enable debug mode, pass the opModes
     *     telemetry object into the constructor of the ConstantsLoader class.
     * </p>
     */
    public void load() {
        for (File file : getConstantsFilesFromOnbotJava()) {
            if (!isTextFile(file)) continue;

            String fileName = file.getName();

            Optional<Class<?>> constantsClass
                    = matchConstantsClassToConstantsFile(stripFileExtension(fileName));

            if (!constantsClass.isPresent()) continue;

            populateClassFromPropertiesFile(constantsClass.get(), fileName);
        }

        if (debug) debugger.displayAll();
    }

    private boolean isTextFile(@NonNull File file) {
        String fileName = file.getName();

        int extensionPosition = fileName.lastIndexOf('.') + 1;

        return fileName.substring(extensionPosition).equals("txt");
    }

    @NonNull private String stripFileExtension(@NonNull String fileName) {
        int extensionOffset = fileName.lastIndexOf('.');

        return fileName.substring(0, extensionOffset);
    }

    private float loadFloat(@NonNull String key, @NonNull Properties properties) {
        return Float.parseFloat(properties.getProperty(key));
    }

    private double loadDouble(
            @NonNull String key,
            @NonNull Properties properties
    ) {
        return Double.parseDouble(properties.getProperty(key));
    }

    private byte loadByte(@NonNull String key, @NonNull Properties properties) {
        return Byte.parseByte(properties.getProperty(key));
    }

    private short loadShort(@NonNull String key, @NonNull Properties properties) {
        return Short.parseShort(properties.getProperty(key));
    }

    private int loadInteger(@NonNull String key, @NonNull Properties properties) {
        return Integer.parseInt(properties.getProperty(key));
    }

    private long loadLong(@NonNull String key, @NonNull Properties properties) {
        return Long.parseLong(properties.getProperty(key));
    }

    private boolean loadBoolean(
            @NonNull String key,
            @NonNull Properties properties
    ) {
        return Boolean.parseBoolean(properties.getProperty(key));
    }

    private char loadCharacter(
            @NonNull String key,
            @NonNull Properties properties
    ) {
       return properties.getProperty(key).toCharArray()[0];
    }

    @NonNull private String loadString(
            @NonNull String key,
            @NonNull Properties properties
    ) {
        return properties.getProperty(key);
    }

    @NonNull private Servo.Direction loadServoDirection(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String servoDirectionString = properties.getProperty(key);

        switch (servoDirectionString.toLowerCase()) {
            case "forward":
                return Servo.Direction.FORWARD;
            case "reverse":
            case "backwards":
                return Servo.Direction.REVERSE;
            default:
                throw new MalformedPropertyException(
                        servoDirectionString,
                        "Failed To Parse Direction",
                        key,
                        debugger
                );
        }
    }

    @NonNull private DcMotorSimple.Direction loadMotorDirection(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String motorDirectionString = properties.getProperty(key);

        switch (motorDirectionString.toLowerCase()) {
            case "forward":
                return DcMotor.Direction.FORWARD;
            case "reverse":
            case "backwards":
                return DcMotorSimple.Direction.REVERSE;
            default:
                throw new MalformedPropertyException(
                        motorDirectionString,
                        "Failed To Parse Direction",
                        key,
                        debugger
                );
        }
    }

    @NonNull private DcMotor.ZeroPowerBehavior loadZeroPowerBehaviour(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String zeroPowerBehaviorString = properties.getProperty(key);

        switch (zeroPowerBehaviorString.toLowerCase()) {
            case "float":
                return DcMotor.ZeroPowerBehavior.FLOAT;
            case "brake":
                return DcMotor.ZeroPowerBehavior.BRAKE;
            case "unknown":
                return DcMotor.ZeroPowerBehavior.UNKNOWN;
            default:
                throw new MalformedPropertyException(
                        zeroPowerBehaviorString,
                        "Failed To Parse ZeroPowerBehavior",
                        key,
                        debugger
                );
        }
    }

    @NonNull private DcMotor.RunMode loadRunMode(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String runModeString = properties.getProperty(key);

        switch (runModeString.toLowerCase()) {
            case "run_to_position":
               return DcMotor.RunMode.RUN_TO_POSITION;
            case "run_using_encoders":
                return DcMotor.RunMode.RUN_USING_ENCODER;
            case "run_without_encoders":
                return DcMotor.RunMode.RUN_WITHOUT_ENCODER;
            case "stop_and_reset_encoders":
                return DcMotor.RunMode.STOP_AND_RESET_ENCODER;
            default:
                throw new MalformedPropertyException(
                        runModeString,
                        "Failed To Parse As RunMode",
                        key,
                        debugger
                );
        }
    }

    @NonNull private UsbFacingDirection loadUsbFacingDirection(
           @NonNull String key,
           @NonNull Properties properties
    ) throws MalformedPropertyException {
        String USBFacingDirectionString = properties.getProperty(key);

        switch (USBFacingDirectionString.toLowerCase()) {
            case "up":
                return UsbFacingDirection.UP;
            case "down":
                return UsbFacingDirection.DOWN;
            case "left":
                return UsbFacingDirection.LEFT;
            case "right":
                return UsbFacingDirection.RIGHT;
            case "backwards":
                return UsbFacingDirection.BACKWARD;
            case "forwards":
                return UsbFacingDirection.FORWARD;
            default:
                throw new MalformedPropertyException(
                       USBFacingDirectionString,
                       "Failed To Parse As UsbFacingDirection",
                       key,
                       debugger
                );
        }
    }

    @NonNull private LogoFacingDirection loadLogoFacingDirection(
       @NonNull String key,
       @NonNull Properties properties
    ) throws MalformedPropertyException {
       String logoFacingString = properties.getProperty(key);

       switch (logoFacingString.toLowerCase()) {
           case "up":
               return LogoFacingDirection.UP;
           case "down":
               return LogoFacingDirection.DOWN;
           case "left":
               return LogoFacingDirection.LEFT;
           case "right":
               return LogoFacingDirection.RIGHT;
           case "forwards":
               return LogoFacingDirection.FORWARD;
           case "backwards":
               return LogoFacingDirection.BACKWARD;
           default:
               throw new MalformedPropertyException(
                       logoFacingString,
                       "Failed To Parse As LogoFacingDirection",
                       key,
                       debugger
               );
       }
    }

    @NonNull private SparkFunOTOS.Pose2D loadSparkFunPose2D(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String poseString = properties.getProperty(key);

        double[] poseValues;

        try {
            poseValues = parseThreePartValue(poseString);
        } catch (NumberFormatException numberFormatException) {
            String message;

            if (numberFormatException.getMessage() == null) {
                message = numberFormatException.getMessage();
            } else {
                message = "Failed To Parse As Number";
            }

            throw new MalformedPropertyException(
                   poseString,
                   message,
                   key,
                   debugger
            );
        }

        return new SparkFunOTOS.Pose2D(poseValues[0], poseValues[1], poseValues[2]);
    }

    @NonNull private Scalar loadScalar(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String scalarString = properties.getProperty(key);

        double[] scalarValues;

        try {
            scalarValues = parseThreePartValue(scalarString);
        } catch (NumberFormatException numberFormatException) {
            String message;

            if (numberFormatException.getMessage() != null) {
                message = numberFormatException.getMessage();
            } else {
                message = "Failed To Parse As Number";
            }

            throw new MalformedPropertyException(
                    scalarString,
                    message,
                    key,
                    debugger
            );
        }

        return new Scalar((int) scalarValues[0], (int) scalarValues[1], (int) scalarValues[2]);
    }

    @NonNull private double[] parseThreePartValue(
            @NonNull String value
    ) throws NumberFormatException {
        int indexOfFirstOpenParentheses   = value.indexOf('(');
        int indexOfFirstClosedParentheses = value.indexOf(')');
        int indexOfFirstComma             = value.indexOf(',');
        int indexOfSecondComma            = value.indexOf(',', indexOfFirstComma + 1);

        String valueOneString
                = value.substring(indexOfFirstOpenParentheses + 1, indexOfFirstComma);
        String valueTwoString
                = value.substring(indexOfFirstComma + 1, indexOfSecondComma + 1);
        String valueThreeString
                = value.substring(indexOfSecondComma + 1, indexOfFirstClosedParentheses);

        double valueOne   = Double.parseDouble(valueOneString);
        double valueTwo   = Double.parseDouble(valueTwoString);
        double valueThree = Double.parseDouble(valueThreeString);

        return new double[]{valueOne, valueTwo, valueThree};
    }

    private boolean isLoadable(@NonNull Field field) {
        int modifiers    = field.getModifiers();
        String fieldName = field.getName();

        boolean fieldIsPublic = Modifier.isPublic(modifiers);
        boolean fieldIsStatic = Modifier.isStatic(modifiers);
        boolean fieldIsFinal  = Modifier.isFinal(modifiers);

        if (!fieldIsPublic || !fieldIsStatic || fieldIsFinal) {
            String message = "Field " + fieldName + " cannot be loaded";

            if (!fieldIsPublic) message += "\nReason: Field Is Not Public";
            if (!fieldIsStatic) message += "\nReason: Field Is No Static";
            if (fieldIsFinal)   message += "\nReason: Field Is Final";

            if (debug) debugger.addMessage(message);
            return false;
        }
        return true;
    }

    private static class MalformedPropertyException extends Exception {
        public static boolean debug = false;

        public MalformedPropertyException(
                @NonNull String value,
                @NonNull String reason,
                @NonNull String name,
                @NonNull Debugger debugger
        ) {
            super("Property " + name + " Cannot Be Loaded");

            if (!debug) return;

            String issue = "Field " + name + " Cannot Be Loaded"
                         + "\nReason: Value [" + value + "] Is Malformed | " + reason;

            debugger.addMessage(issue);
        }
    }

    private static class Debugger {
        private final ArrayList<String> output;
        private final Telemetry telemetry;

        public Debugger(@Nullable Telemetry telemetry) {
            this.output    = new ArrayList<>();
            this.telemetry = telemetry;
        }

        public void addMessage(@NonNull String message) {
            output.add(message);
        }

        public void displayAll() {
            if (telemetry == null) return;

            telemetry.addData("Number Of Issues", output.size());

            for (String message: output) { telemetry.addLine("\n" + message); }
        }
    }
}
