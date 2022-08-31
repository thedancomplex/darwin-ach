from dynamixel_sdk import * 
import numpy as np
import time

class MX28:
    # MX28 Control Table
    P_MODEL_NUMBER_L = 0
    P_MODEL_NUMBER_H = 1
    P_VERSION_OF_FIRMWARE = 2
    P_ID = 3
    P_BAUD_RATE = 4
    P_RETURN_DELAY_TIME = 5
    P_CW_ANGLE_LIMIT_L = 6
    P_CW_ANGLE_LIMIT_H = 7
    P_CCW_ANGLE_LIMIT_L = 8
    P_CCW_ANGLE_LIMIT_H = 9
    P_HIGHEST_LIMIT_TEMPERATURE = 11
    P_LOWEST_LIMIT_VOLTAGE = 12
    P_HIGHEST_LIMIT_VOLTAGE = 13
    P_MAX_TORQUE_L = 14
    P_MAX_TORQUE_H = 15
    P_STATUS_RETURN_LEVEL = 16
    P_ALARM_LED = 17
    P_ALARM_SHUTDOWN = 18
    P_TORQUE_ENABLE = 24
    P_LED = 25
    P_D_GAIN = 26
    P_I_GAIN = 27
    P_P_GAIN = 28
    P_GOAL_POSITION_L = 30
    P_GOAL_POSITION_H = 31
    P_MOVING_SPEED_L = 32
    P_MOVING_SPEED_H = 33
    P_TORQUE_LIMIT_L = 34
    P_TORQUE_LIMIT_H = 35
    P_PRESENT_POSITION_L = 36
    P_PRESENT_POSITION_H = 37
    P_PRESENT_SPEED_L = 38
    P_PRESENT_SPEED_H = 39 
    P_PRESENT_LOAD_L = 40
    P_PRESENT_LOAD_H = 41
    P_PRESENT_VOLTAGE = 42
    P_PRESENT_TEMPERATURE = 43
    P_REGISTERED_INSTRUCTION = 44
    P_MOVING = 46
    P_LOCK = 47
    P_PUNCH_L = 48
    P_PUNCH_H = 49
    P_CURRENT_L = 68
    P_CURRENT_H = 69
    P_TORQUE_CONTROL_MODE_ENABLE = 70
    P_GOAL_TORQUE_L = 71
    P_GOAL_TORQUE_H = 72
    P_GOAL_ACCELERATION = 73

    def __init__(self):
        pass

class CM730:
    CM730_MODEL_NUMBER = 29440
    MX28_MODEL_NUMBER = 29
    CM730_ID = 200
    # CM730 Control Table
    P_MODEL_NUMBER_L = 0
    P_MODEL_NUMBER_H = 1
    P_VERSION = 2
    P_ID = 3
    P_BAUD_RATE = 4
    P_RETURN_DELAY_TIME = 5
    P_RETURN_LEVEL = 16
    P_DXL_POWER = 24
    P_LED_PANNEL = 25
    P_LED_HEAD_L = 26
    P_LED_HEAD_H = 27
    P_LED_EYE_L = 28
    P_LED_EYE_H = 29
    P_BUTTON = 30
    P_GYRO_Z_L = 38
    P_GYRO_Z_H = 39
    P_GYRO_Y_L = 40
    P_GYRO_Y_H = 41
    P_GYRO_X_L = 42
    P_GYRO_X_H = 43
    P_ACCEL_X_L = 44
    P_ACCEL_X_H = 45
    P_ACCEL_Y_L = 46
    P_ACCEL_Y_H = 47
    P_ACCEL_Z_L = 48
    P_ACCEL_Z_H = 49
    P_VOLTAGE = 50
    P_LEFT_MIC_L = 51
    P_LEFT_MIC_H = 52
    P_ADC2_L = 53
    P_ADC2_H = 54
    P_ADC3_L = 55
    P_ADC3_H = 56
    P_ADC4_L = 57
    P_ADC4_H = 58
    P_ADC5_L = 59
    P_ADC5_H = 60
    P_ADC6_L = 61
    P_ADC6_H = 62
    P_ADC7_L = 63
    P_ADC7_H = 64
    P_ADC8_L = 65
    P_ADC8_H = 66
    P_RIGHT_MIC_L = 67
    P_RIGHT_MIC_H = 68
    P_ADC10_L = 69
    P_ADC10_H = 70
    P_ADC11_L = 71
    P_ADC11_H = 72	
    P_ADC12_L = 73
    P_ADC12_H = 74
    P_ADC13_L = 75
    P_ADC13_H = 76
    P_ADC14_L = 77
    P_ADC14_H = 78
    P_ADC15_L = 79
    P_ADC15_H = 80

    model_dict = {
        CM730_MODEL_NUMBER: "CM730",
        MX28_MODEL_NUMBER: "MX28"
    }

    def __init__(self):
        self.DEBUG_MODE = False
        self.port = "/dev/ttyUSB0"
        self.baudrate = 1000000
        self.protocol = 1
        self.portHandler = PortHandler(self.port)
        self.packetHandler = PacketHandler(self.protocol)
        self.mx28 = MX28()

    def scaling(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def connect(self):
        # Opening port
        ret = self.portHandler.openPort()
        if ret == True:
            print("Success open port")
        else:
            print("Fail open port")
        # Set port baudrate
        ret = self.portHandler.setBaudRate(self.baudrate)
        if ret == True:
            print("Success change baudrate")
        else:
            print("Fail change baudrate")
        # Check CM730
        self.check_cm730()

    def disconnect(self):
        self.portHandler.closePort()
        print("Port closed")
    
    def check_cm730(self):
        model_number, comm_result, error = self.packetHandler.ping(self.portHandler, self.CM730_ID)
        if self.DEBUG_MODE:
            if comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(comm_result))
            elif error != 0:
                print("%s" % self.packetHandler.getRxPacketError(error))
        if model_number == self.CM730_MODEL_NUMBER:
            print("CM730 Found")
        else:
            print("CM730 Not Found")
    
    # Oke
    def ping(self, ID):
        model_number, comm_result, error = self.packetHandler.ping(self.portHandler, ID)
        if self.DEBUG_MODE:
            if comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(comm_result))
            elif error != 0:
                print("%s" % self.packetHandler.getRxPacketError(error))
        return model_number
    
    # Oke
    def read_byte(self, ID, address):
        value, comm_result, error = self.packetHandler.read1ByteTxRx(self.portHandler, ID, address)
        if self.DEBUG_MODE:
            if comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(comm_result))
            elif error != 0:
                print("%s" % self.packetHandler.getRxPacketError(error))
        return value
    
    # Oke
    def read_word(self, ID, address):
        value, comm_result, error = self.packetHandler.read2ByteTxRx(self.portHandler, ID, address)
        if self.DEBUG_MODE:
            if comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(comm_result))
            elif error != 0:
                print("%s" % self.packetHandler.getRxPacketError(error))
        return value
    
    # Oke
    def read_double_word(self, ID, address):
        value, comm_result, error = self.packetHandler.read4ByteTxRx(self.portHandler, ID, address)
        if self.DEBUG_MODE:
            if comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(comm_result))
            elif error != 0:
                print("%s" % self.packetHandler.getRxPacketError(error))
        return value

    # Oke
    def write_byte(self, ID, address, value):
        err = False
        comm_result, error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, address, value)
        if comm_result != COMM_SUCCESS:
            err = True
        elif error != 0:
            err = True
        if self.DEBUG_MODE:
            if comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(comm_result))
            elif error != 0:
                print("%s" % self.packetHandler.getRxPacketError(error))
        return err

    # Oke
    def write_word(self, ID, address, value):
        err = False
        comm_result, error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, address, value)
        if comm_result != COMM_SUCCESS:
            err = True
        elif error != 0:
            err = True
        if self.DEBUG_MODE:
            if comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(comm_result))
            elif error != 0:
                print("%s" % self.packetHandler.getRxPacketError(error))
        return err
    
    def write_double_word(self, ID, address, value):
        err = False
        comm_result, error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, address, value)
        if comm_result != COMM_SUCCESS:
            err = True
        elif error != 0:
            err = True
        if self.DEBUG_MODE:
            if comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(comm_result))
            elif error != 0:
                print("%s" % self.packetHandler.getRxPacketError(error))
        return err

    def check_ID(self, start_ID, end_ID):
        for i in range(start_ID, end_ID):
            model = self.ping(i)
            if model != 0:
                print("Found ID : " + str(i) + " Model : " + self.model_dict[model])

    def dxl_on(self):
        error = self.write_byte(self.CM730_ID, self.P_DXL_POWER, 1)
        return error 
    
    def dxl_off(self):
        error = self.write_byte(self.CM730_ID, self.P_DXL_POWER, 0)
        return error
    
    def read_dxl_power(self):
        value = self.read_byte(self.CM730_ID, self.P_DXL_POWER)
        return value

    def read_led_pannel(self):
        value = self.read_byte(self.CM730_ID, self.P_LED_PANNEL)
        return value
    
    def write_led_pannel(self, value):
        error = self.write_byte(self.CM730_ID, self.P_LED_PANNEL, value)
        return error

    def read_led_head(self):
        value = self.read_word(self.CM730_ID, self.P_LED_HEAD_L)
        return value
    
    def write_led_head(self, value):
        error = self.write_word(self.CM730_ID, self.P_LED_HEAD_L, value)
        return error

    def read_led_eye(self):
        value = self.read_word(self.CM730_ID, self.P_LED_EYE_L)
        return value
    
    def write_led_eye(self, value):
        error = self.write_word(self.CM730_ID, self.P_LED_EYE_L, value)
        return error

    def read_button(self):
        right_button = False
        centre_button = False
        value = self.read_byte(self.CM730_ID, self.P_BUTTON)
        if value == 3:
            right_button = True
            centre_button = True
        elif value == 2:
            centre_button = True
        elif value == 1:
            right_button = True
        return centre_button, right_button
    
    def read_gyro(self, in_dps=True):
        raw_x = self.read_word(self.CM730_ID, self.P_GYRO_X_L)
        raw_y = self.read_word(self.CM730_ID, self.P_GYRO_Y_L)
        raw_z = self.read_word(self.CM730_ID, self.P_GYRO_Z_L)
        if in_dps:
            raw_x = self.scaling(raw_x, 0.0, 1023.0, -500.0, 500.0)
            raw_y = self.scaling(raw_y, 0.0, 1023.0, -500.0, 500.0)
            raw_z = self.scaling(raw_z, 0.0, 1023.0, -500.0, 500.0)
        return raw_x, raw_y, raw_z

    def read_accelerometer(self, in_g=True):
        raw_x = self.read_word(self.CM730_ID, self.P_ACCEL_X_L)
        raw_y = self.read_word(self.CM730_ID, self.P_ACCEL_Y_L)
        raw_z = self.read_word(self.CM730_ID, self.P_ACCEL_Z_L)
        if in_g:
            raw_x = self.scaling(raw_x, 0.0, 1023.0, -4.0, 4.0)
            raw_y = self.scaling(raw_y, 0.0, 1023.0, -4.0, 4.0)
            raw_z = self.scaling(raw_z, 0.0, 1023.0, -4.0, 4.0)
        return raw_x, raw_y, raw_z

    def read_voltage(self, in_volt=True):
        value = self.read_byte(self.CM730_ID, self.P_VOLTAGE)
        if in_volt:
            value = value / 10.0
        return value

    # Single servo access
    def servo_write_max_torque(self, ID, value):
        error = self.write_word(ID, self.mx28.P_MAX_TORQUE_L, value)
        return error

    def servo_read_max_torque(self, ID):
        value = self.read_word(ID, self.mx28.P_MAX_TORQUE_L)
        return value

    def servo_enable_torque(self, ID):
        error = self.write_byte(ID, self.mx28.P_TORQUE_ENABLE, 1)
        return error
    
    def servo_disable_torque(self, ID):
        error = self.write_byte(ID, self.mx28.P_TORQUE_ENABLE, 0)
        return error
    
    def servo_write_position(self, ID, value, in_radians=True):
        if in_radians:
            value = int(self.scaling(value, -np.pi, np.pi, 0, 4095))
        error = self.write_word(ID, self.mx28.P_GOAL_POSITION_L, value)
        return error
    
    def servo_read_position(self, ID, in_radians=True):
        value = self.read_word(ID, self.mx28.P_PRESENT_POSITION_L)
        if in_radians:
            value = self.scaling(value, 0, 4095, -np.pi, np.pi, )
        return value

    def servo_write_speed(self, ID, value, in_rpm=True):
        if in_rpm:
            value = int(self.scaling(value, 0.0, 116.62, 0, 1023))
        error = self.write_word(ID, self.mx28.P_MOVING_SPEED_L, value)
        return error
    
    def servo_read_speed(self, ID, in_rpm=True):
        value = self.read_word(ID, self.mx28.P_PRESENT_SPEED_L)
        if in_rpm:
            value = self.scaling(value, 0, 1023, 0.0, 116.62)
        return value

    def servo_write_torque(self, ID, value, in_percent=True):
        if in_percent:
            value = int(self.scaling(value, 0.0, 100.0, 0, 1023))
        error = self.write_word(ID, self.mx28.P_TORQUE_LIMIT_L, value)
        return error
    
    def servo_read_torque(self, ID, in_percent=True):
        value = self.read_word(ID, self.mx28.P_PRESENT_LOAD_L)
        if in_percent:
            # CCW Direction
            if value >= 0 and value <= 1023:
                value = self.scaling(value, 0, 1023, 0, -100.0)
            # CW Direction
            elif value >= 1024 and value <= 2047:
                value = self.scaling(value, 1024, 2047, 0, 100.0)
        return value

    def servo_read_voltage(self, ID, in_volt=True):
        value = self.read_byte(ID, self.mx28.P_PRESENT_VOLTAGE)
        if in_volt:
            value = value / 10.0
        return value
    
    def servo_read_temperature(self, ID):
        value = self.read_word(ID, self.mx28.P_PRESENT_TEMPERATURE)
        return value

    # Multiple servo access
    def servo_sync_write(self, list_ID, list_value, start_address, length):
        error = False
        groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, start_address, length)
        for i in range(len(list_ID)):
            if length == 1:
                param = [list_value[i]]
            elif length == 2:
                param = [DXL_LOBYTE(list_value[i]), DXL_HIBYTE(list_value[i])]
            addparam_result = groupSyncWrite.addParam(list_ID[i], param)
            if addparam_result != True:
                error = True
            if self.DEBUG_MODE:
                if addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % list_ID[i])
        comm_result = groupSyncWrite.txPacket()
        if comm_result != COMM_SUCCESS:
            error = True
        if self.DEBUG_MODE:
            if comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(comm_result))
        groupSyncWrite.clearParam()
        return error

    def servo_bulk_read(self, list_ID, start_address, length):
        error = False
        list_value = []
        groupBulkRead = GroupBulkRead(self.portHandler, self.packetHandler)
        for i in range(len(list_ID)):
            addparam_result = groupBulkRead.addParam(list_ID[i], start_address, length)
            if addparam_result != True:
                error = True 
            if self.DEBUG_MODE:
                if addparam_result != True:
                    print("[ID:%03d] groupBulkRead addparam failed" % list_ID[i])
        comm_result = groupBulkRead.txRxPacket()
        if comm_result != COMM_SUCCESS:
            error = True
        if self.DEBUG_MODE:
            if comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(comm_result))
        for i in range(len(list_ID)):
            value = groupBulkRead.getData(list_ID[i], start_address, length)
            list_value.append(value)
        groupBulkRead.clearParam()
        return list_value, error

    def servo_sync_enable_torque(self, list_ID):
        error = self.servo_sync_write(list_ID, [1]*len(list_ID), self.mx28.P_TORQUE_ENABLE, 1)
        return error

    def servo_sync_disable_torque(self, list_ID):
        error = self.servo_sync_write(list_ID, [0]*len(list_ID), self.mx28.P_TORQUE_ENABLE, 1)
        return error

    def servo_sync_write_position(self, list_ID, list_value, in_angle=True):
        if in_angle:
            for i in range(len(list_value)):
                list_value[i] = int(self.scaling(list_value[i], -np.pi, np.pi, 0, 4095))
        error = self.servo_sync_write(list_ID, list_value, self.mx28.P_GOAL_POSITION_L, 2)
        return error

    def servo_bulk_read_position(self, list_ID, in_angle=True):
        list_value, error = self.servo_bulk_read(list_ID, self.mx28.P_PRESENT_POSITION_L, 2)
        if in_angle:
            for i in range(len(list_value)):
                list_value[i] = self.scaling(list_value[i], 0, 4095, -np.pi, np.pi)
        return list_value, error

    def servo_sync_write_speed(self, list_ID, list_value, in_rpm=True):
        if in_rpm:
            for i in range(len(list_value)):
                list_value[i] = int(self.scaling(list_value[i], 0.0, 116.62, 0, 1023))
        error = self.servo_sync_write(list_ID, list_value, self.mx28.P_PRESENT_SPEED_L, 2)
        return error

    def servo_bulk_read_speed(self, list_ID, in_rpm=True):
        list_value, error = self.servo_bulk_read(list_ID, self.mx28.P_PRESENT_SPEED_L, 2)
        if in_rpm:
            for i in range(len(list_value)):
                list_value[i] = self.scaling(list_value[i], 0, 1023, 0.0, 116.62)
        return list_value, error

    def servo_sync_write_torque(self, list_ID, list_value, in_percent=True):
        if in_percent:
            for i in range(len(list_value)):
                list_value[i] = int(self.scaling(list_value[i], 0.0, 100.0, 0, 1023))
        error = self.servo_sync_write(list_ID, list_value, self.mx28.P_TORQUE_LIMIT_L, 2)
        return error

    def servo_bulk_read_torque(self, list_ID, in_percent=True):
        list_value, error = self.servo_bulk_read(list_ID, self.mx28.P_PRESENT_LOAD_L, 2)
        if in_percent:
            for i in range(len(list_value)):
                # CCW Direction
                if list_value[i] >= 0 and list_value[i] <= 1023:
                    list_value[i] = self.scaling(list_value[i], 0, 1023, 0, -100.0)
                # CW Direction
                elif list_value[i] >= 1024 and list_value[i] <= 2047:
                    list_value[i] = self.scaling(list_value[i], 1024, 2047, 0, 100.0)
        return list_value, error

    def servo_bulk_read_voltage(self, list_ID, in_volt=True):
        list_value, error = self.servo_bulk_read(list_ID, self.mx28.P_PRESENT_VOLTAGE, 1)
        if in_volt:
            for i in range(len(list_value)):
                list_value[i] = list_value[i] / 10.0
        return list_value, error

    # temperature already in celcius
    def servo_bulk_read_temperature(self, list_ID):
        list_value, error = self.servo_bulk_read(list_ID, self.mx28.P_PRESENT_TEMPERATURE, 2)
        return list_value, error

    def all_servo_enable_torque(self):
        list_ID = [i for i in range(23)]
        self.servo_sync_enable_torque(list_ID)

    def all_servo_disable_torque(self):
        list_ID = [i for i in range(23)]
        self.servo_sync_disable_torque(list_ID)

def main():
    cm730 = CM730()
    cm730.connect()
    cm730.dxl_on()
    time.sleep(1)
    cm730.check_ID(0, 255)
    cm730.servo_sync_enable_torque([19, 20])
    for i in range(0, 100, 1):
        print(cm730.read_button())
        print(cm730.servo_bulk_read_position([19, 20]))
        print(cm730.servo_bulk_read_speed([19, 20]))
        print(cm730.servo_bulk_read_torque([19, 20]))
        print(cm730.servo_bulk_read_voltage([19, 20]))
        print(cm730.servo_bulk_read_temperature([19, 20]))
        time.sleep(0.5)
    cm730.all_servo_disable_torque()
    cm730.disconnect()
    
if __name__ == "__main__":
    main()