package org.firstinspires.ftc.teamcode.drivers;

import static com.qualcomm.robotcore.util.Util.concatenateByteArrays;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.R;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

@I2cDeviceType
@DeviceProperties(name = "@string/dfrobot_huskylens_name", description = "@string/dfrobot_huskylens_description", xmlTag = "HuskyLens")
public class CustomHuskyLens extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    private final String TAG = "HuskyLens";

    private final byte CMD_REQUEST_KNOCK = 0x2C;
    private final byte CMD_REQUEST_ALL = 0x20;       // unimplemented
    private final byte CMD_REQUEST_BLOCKS = 0x21;
    private final byte CMD_REQUEST_BLOCK_BY_ID = 0x27;
    private final byte CMD_REQUEST_ARROWS = 0x22;
    private final byte CMD_REQUEST_ARROW_BY_ID = 0x28;
    private final byte CMD_REQUEST_ALGORITHM = 0x2D;

    private final byte LEARNED = 0x23;               // unimplemented
    private final byte BLOCKS_LEARNED = 0x24;
    private final byte ARROWS_LEARNED = 0x25;
    private final byte BY_ID = 0x26;
    private final byte BLOCKS_BY_ID = 0x27;
    private final byte ARROWS_BY_ID = 0x28;

    private final byte CMD_RESP_OK = 0x2E;
    private final byte CMD_RESP_INFO = 0x29;
    private final byte CMD_RESP_BLOCK = 0x2A;
    private final byte CMD_RESP_ARROW = 0x2B;

    private final byte FACE_RECOGNITION = 0x00;
    private final byte OBJECT_TRACKING = 0x01;
    private final byte OBJECT_RECOGNITION = 0x02;
    private final byte LINE_TRACKING = 0x03;
    private final byte COLOR_RECOGNITION = 0x04;
    private final byte TAG_RECOGNITION = 0x05;
    private final byte OBJECT_CLASSIFICATION = 0x06;

    private final byte HEADER_BYTE_1 = 0x55;
    private final byte HEADER_BYTE_2 = (byte)0xAA;
    private final byte PROTOCOL_ADDR = 0x11;

    protected final int DEFAULT_I2C_ADDR = 0x32;

    private final byte CMD_RESP_INFO_LEN = 16;
    private final byte CMD_RESP_BLOCK_LEN = 16;
    private final byte CMD_RESP_ARROW_LEN = 16;
    private final byte CMD_RESP_OK_LEN = 6;

    private final byte MAX_BLOCKS_IN_RESPONSE = 8;
    private final byte MAX_ARROWS_IN_RESPONSE = 6;

    private static final Block[] NO_BLOCKS = {};
    private static final Arrow[] NO_ARROWS = {};

    private static final boolean debug = false;

    /**
     * Algorithms to be used with selectAlgorithm()
     */
    public enum Algorithm {
        FACE_RECOGNITION(0x00),
        OBJECT_TRACKING(0x01),
        OBJECT_RECOGNITION(0x02),
        LINE_TRACKING(0x03),
        COLOR_RECOGNITION(0x04),
        TAG_RECOGNITION(0x05),
        OBJECT_CLASSIFICATION(0x06),

        NONE(0xFF);

        public byte bVal;
        Algorithm(int val) { this.bVal = (byte)val; }
    }

    /**
     * A Block is a fundamental unit of recognition for all recognized kinds except Arrows and describes
     * the recognized entity's location center, width, height, top left corner, and id.
     *
     * Units are pixels, except for the id.  The device's resolution is 320x240.
     */
    public class Block {
        public final int x;
        public final int y;
        public final int width;
        public final int height;
        public final int top;
        public final int left;
        public final int id;

        public Block(byte[] buf) {
            if (buf == null) throw new IllegalArgumentException();
            if (buf.length < 16) throw new IllegalArgumentException();

            this.x = byteToShort(buf, 5);
            this.y = byteToShort(buf, 7);
            this.width = byteToShort(buf, 9);
            this.height = byteToShort(buf, 11);
            this.id = byteToShort(buf, 13);

            this.top = this.y - (this.height / 2);
            this.left = this.x - (this.width / 2);
        }

        public String toString() {
            return "id=" + id + " size: " + width + "x" + height + " position: " + x + "," + y;
        }
    }

    /**
     * An Arrow is returned when the algorithm is set to LINE_TRACKING and represents
     * the direction the line faces.
     *
     * Units are pixels, except for the id.  The device's resolution is 320x240.
     */
    public class Arrow {
        public final int x_origin;
        public final int y_origin;
        public final int x_target;
        public final int y_target;
        public final int id;

        public Arrow(byte[] buf) {
            if (buf == null) throw new IllegalArgumentException();
            if (buf.length < 16) throw new IllegalArgumentException();

            this.x_origin = byteToShort(buf, 5);
            this.y_origin = byteToShort(buf, 7);
            this.x_target = byteToShort(buf, 9);
            this.y_target = byteToShort(buf, 11);
            this.id = byteToShort(buf, 13);
        }

        public String toString() {
            return "id=" + id + " origin:" + x_origin + "," + y_origin + " target:" + x_target + "," + y_target;
        }
    }

    public CustomHuskyLens(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_I2C_ADDR));
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.DFRobot;
    }

    @Override
    protected synchronized boolean doInitialize() {
        /*
         * Not setting a default algorithm here because algorithms are selectable
         * on the device itself and we should not override an algorithm if the
         * user selected one externally.
         */
        return knock();
    }

    @Override
    public String getDeviceName() {
        return AppUtil.getDefContext().getString(R.string.dfrobot_huskylens_name);
    }

    /**
     * Proof of life check.
     *
     * @return true if the device responds, false otherwise.
     */
    public boolean knock() {
        sendCommand(CMD_REQUEST_KNOCK);
        byte[] buf = this.deviceClient.read(CMD_RESP_OK_LEN);
        if (debug) RobotLog.logBytes(TAG, "knock resp", buf, buf.length);
        if (buf == null) {
            RobotLog.ee(TAG, "I2C transaction failed");
            return false;
        } else if (!verifyChecksum(buf)) {
            RobotLog.ee(TAG, "Checksum failed for command KNOCK ");
            return false;
        } else {
            return true;
        }
    }

    /**
     * Select the algorithm that the HuskyLens should use.  This should be called
     * upon startup to ensure that the device is returning what you expect it to return.
     */
    public void selectAlgorithm(Algorithm algorithm) {
        sendCommandWithData(CMD_REQUEST_ALGORITHM, algorithm.bVal, (byte)0x00);
    }

    /**
     * Returns an array of blocks or the empty array if no blocks are seen.
     */
    public Block[] blocks() {
        sendCommand(CMD_REQUEST_BLOCKS);
        return readBlocksResponse();
    }

    /**
     * Returns an array of blocks with the given id or the empty array if no blocks are seen.
     */
    public Block[] blocks(int id) {
        byte highByte = (byte)(id & 0xFF);
        byte lowByte = (byte)((id >> 8) & 0xFF);
        sendCommandWithData(CMD_REQUEST_BLOCK_BY_ID, highByte, lowByte);

        return readBlocksResponse();
    }

    /**
     * Returns an array of arrows or the empty array if no arrows are seen.
     */
    public Arrow[] arrows() {
        sendCommand(CMD_REQUEST_ARROWS);
        return readArrowsResponse();
    }

    /**
     * Returns an array of arrows with the given id or the empty array if no arrows are seen.
     */
    public Arrow[] arrows(int id) {
        byte highByte = (byte)(id & 0xFF);
        byte lowByte = (byte)((id >> 8) & 0xFF);
        sendCommandWithData(CMD_REQUEST_ARROW_BY_ID, highByte, lowByte);

        return readArrowsResponse();
    }

    protected byte[] readInfo() {
        return this.deviceClient.read(0, CMD_RESP_INFO_LEN);
    }

    protected Block[] readBlocksResponse() {
        byte[] info = readInfo();

        if (!verifyChecksum(info)) {
            return NO_BLOCKS;
        }

        if (debug) RobotLog.logBytes(TAG, "info resp", info, info.length);
        int count = byteToShort(info, 5);
        if (count == 0) {
            return NO_BLOCKS;
        }
        if (count > MAX_BLOCKS_IN_RESPONSE) {
            count = MAX_BLOCKS_IN_RESPONSE;
        }

        Block[] blocks = new Block[count];
        byte[] block_buf = new byte[CMD_RESP_BLOCK_LEN];

        /*
         * Fix the EH firmware bug.
         *
         * The idea is that instead of trying to work with missing data, given that the
         * way software is structured we also know what byte is missing, simply put that
         * byte back in the buffer where it belongs and pretend that we received a full,
         * complete, and correct message from here on out.
         */
        byte buf[] = this.deviceClient.read((CMD_RESP_BLOCK_LEN * count) - 1);
        byte[] prependByte = {0x55};
        ByteBuffer byteBuf = ByteBuffer.wrap(concatenateByteArrays(prependByte, buf));
        if (debug) RobotLog.logBytes(TAG, "blocks", byteBuf.array(), byteBuf.array().length);

        for (int i = 0; i < count; i++) {
            byteBuf.get(block_buf, 0, CMD_RESP_BLOCK_LEN);
            if (debug) RobotLog.logBytes(TAG, "single block", block_buf, block_buf.length);
            if (!verifyChecksum(block_buf)) {
                RobotLog.ee(TAG, "Checksum failed for block " + i);
                RobotLog.logBytes(TAG, "Checksum failure: ", block_buf, block_buf.length);
                continue;
            }
            blocks[i] = new CustomHuskyLens.Block(block_buf);
            RobotLog.ii(TAG, blocks[i].toString());
        }

        return blocks;
    }

    protected Arrow[] readArrowsResponse()
    {
        byte[] info = readInfo();

        if (!verifyChecksum(info)) {
            return NO_ARROWS;
        }

        if (debug) RobotLog.logBytes(TAG, "info resp", info, info.length);
        int count = byteToShort(info, 5);
        if (count == 0) {
            return NO_ARROWS;
        }
        if (count > MAX_ARROWS_IN_RESPONSE) {
            count = MAX_ARROWS_IN_RESPONSE;
        }

        Arrow[] arrows = new Arrow[count];
        byte[] arrow_buf = new byte[CMD_RESP_ARROW_LEN];

        /*
         * Fix the EH firmware bug.  See comment in readBlocksResponse()
         */
        byte buf[] = this.deviceClient.read((CMD_RESP_ARROW_LEN * count) - 1);
        byte[] prependByte = {0x55};
        ByteBuffer byteBuf = ByteBuffer.wrap(concatenateByteArrays(prependByte, buf));
        if (debug) RobotLog.logBytes(TAG, "arrows", byteBuf.array(), byteBuf.array().length);

        for (int i = 0; i < count; i++) {
            byteBuf.get(arrow_buf, 0, CMD_RESP_ARROW_LEN);
            if (debug) RobotLog.logBytes(TAG, "single arrow", arrow_buf, arrow_buf.length);
            if (!verifyChecksum(arrow_buf)) {
                RobotLog.ee(TAG, "Checksum failed for arrow " + i);
                RobotLog.logBytes(TAG, "Checksum failure: ", arrow_buf, arrow_buf.length);
                continue;
            }
            arrows[i] = new Arrow(arrow_buf);
            RobotLog.ii(TAG, arrows[i].toString());
        }

        return arrows;
    }

    /*
     * Send a basic command that takes no parameters
     */
    protected void sendCommand(byte cmd) {
        byte[] tmp = { HEADER_BYTE_1, HEADER_BYTE_2, PROTOCOL_ADDR, 0x00, cmd, 0x00};
        ByteBuffer buf = ByteBuffer.wrap(tmp);
        buf.put(5, calculateChecksum(tmp));

        this.deviceClient.write(buf.array());
    }

    protected void sendCommandWithData(byte cmd, byte d1, byte d2) {
        byte[] tmp = { HEADER_BYTE_1, HEADER_BYTE_2, PROTOCOL_ADDR, 0x02, cmd, d1, d2, 0x00};
        ByteBuffer buf = ByteBuffer.wrap(tmp);
        buf.put(7, calculateChecksum(tmp));

        this.deviceClient.write(buf.array());
    }

    private int byteToShort(byte[] m, int i)
    {
        if ((i + 1) > m.length) {
            throw new IllegalArgumentException();
        }
        return TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(m, i, ByteOrder.LITTLE_ENDIAN));
    }

    private byte calculateChecksum(byte[] buf)
    {
        int checksum = 0;

        for (int i = 0; i < buf.length - 1; i++) {
            checksum += (buf[i] & 0xff);
        }

        return (byte)checksum;
    }

    private boolean verifyChecksum(byte[] buf)
    {
        byte sum = calculateChecksum(buf);
        if (sum == buf[buf.length - 1]) {
            return true;
        } else {
            return true;
        }
    }
}