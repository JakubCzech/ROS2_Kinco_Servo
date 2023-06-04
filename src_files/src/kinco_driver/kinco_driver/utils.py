from enum import Enum

ENCODER_RESOLUTION = 10000


class Adress(Enum):
    Controlword = bytearray([0x40, 0x60, 0x00])
    Statusword = bytearray([0x41, 0x60, 0x00])
    OperationMode = bytearray([0x60, 0x60, 0x00])
    AbsRelPosCtrlSel = bytearray([0x20, 0x20, 0x0F])
    PosActual = bytearray([0x63, 0x60, 0x00])
    RealCurrent = bytearray([0x78, 0x60, 0x00])
    StatusInput = bytearray([0xFD, 0x60, 0x00])
    RealSpeed = bytearray([0x6C, 0x60, 0x00])
    InvertDirection = bytearray([0x7E, 0x60, 0x00])
    TargetPosition = bytearray([0x7A, 0x60, 0x00])
    ProfileSpeed = bytearray([0x81, 0x60, 0x00])
    TargetSpeed = bytearray([0xFF, 0x60, 0x00])
    MaxSpeed = bytearray([0x00, 0x60, 0x00])
    ProfileAcc = bytearray([0x03, 0x60, 0x00])
    ProfileDec = bytearray([0x04, 0x60, 0x00])
    TargetTorque = bytearray([0x71, 0x60, 0x00])
    GroupCurrentLoop = bytearray([0xF6, 0x08, 0x10])
    MaxCurrentCommand = bytearray([0x73, 0x60, 0x00])

    Din_Pos0 = bytearray([0x20, 0x20, 0x01])
    Din_Pos1 = bytearray([0x20, 0x20, 0x02])
    Din_Pos2 = bytearray([0x20, 0x20, 0x03])
    Din_Pos3 = bytearray([0x20, 0x20, 0x04])
    Din_Pos4 = bytearray([0x20, 0x20, 0x10])
    Din_Pos5 = bytearray([0x20, 0x20, 0x11])
    Din_Pos6 = bytearray([0x20, 0x20, 0x12])
    Din_Pos7 = bytearray([0x20, 0x20, 0x13])
    Din_Speed0 = bytearray([0x20, 0x20, 0x05])
    Din_Speed1 = bytearray([0x20, 0x20, 0x06])
    Din_Speed2 = bytearray([0x20, 0x20, 0x07])
    Din_Speed3 = bytearray([0x20, 0x20, 0x08])
    Din_Speed4 = bytearray([0x20, 0x20, 0x14])
    Din_Speed5 = bytearray([0x20, 0x20, 0x15])
    Din_Speed6 = bytearray([0x20, 0x20, 0x16])
    Din_Speed7 = bytearray([0x20, 0x20, 0x17])

    Max_Following_Error = bytearray([0x65, 0x60, 0x00])
    Target_Pos_Window = bytearray([0x67, 0x60, 0x00])
    Position_Window_time = bytearray([0x25, 0x08, 0x09, 0x16])
    Target_Speed_Window = bytearray([0x08, 0x25, 0x09])
    Zero_Speed_Window = bytearray([0x10, 0x20, 0x18])
    Zero_Speed_Time = bytearray([0xF9, 0x60, 0x14])
    Soft_Positive_Limit = bytearray([0x60, 0x7D, 0x01])
    Soft_Negative_Limit = bytearray([0x60, 0x7D, 0x02])
    Limit_function = bytearray([0x10, 0x20, 0x19])

    Homing_Method = bytearray([0x98, 0x60, 0x00])
    Homing_Speed_Switch = bytearray([0x99, 0x60, 0x01])
    Homing_Speed_Zero = bytearray([0x99, 0x60, 0x02])
    Homing_Acceleration = bytearray([0x9A, 0x60, 0x00])
    Homing_Offset = bytearray([0x7C, 0x60, 0x00])
    Homing_Offset_Mode = bytearray([0x99, 0x60, 0x05])

    Kvp = bytearray([0xF9, 0x60, 0x01])
    Kvi = bytearray([0xF9, 0x60, 0x02])
    Kvi_32 = bytearray([0xF9, 0x60, 0x07])
    Speed_Fb_N = bytearray([0xF9, 0x60, 0x05])

    Kpp = bytearray([0xFB, 0x60, 0x01])
    K_Velocity_FF = bytearray([0xFB, 0x60, 0x02])
    K_Acc_FF = bytearray([0xFB, 0x60, 0x03])
    Pos_Filter_N = bytearray([0xFB, 0x60, 0x05])

    Din_1_Function = bytearray([0x10, 0x20, 0x03])
    Din_2_Function = bytearray([0x10, 0x20, 0x04])
    Din_3_Function = bytearray([0x10, 0x20, 0x05])
    Din_4_Function = bytearray([0x10, 0x20, 0x06])
    Dout_1_Function = bytearray([0x10, 0x20, 0x0F])
    Dout_2_Function = bytearray([0x10, 0x20, 0x10])
    Din_Real = bytearray([0x10, 0x20, 0x0A])
    Dout_Real = bytearray([0x10, 0x20, 0x14])
    Din_Polarity = bytearray([0x10, 0x20, 0x01])
    Dout_Polarity = bytearray([0x10, 0x20, 0x0D])
    Din_Simulate = bytearray([0x10, 0x20, 0x02])
    Dout_Simulate = bytearray([0x10, 0x20, 0x0E])

    Error_State = bytearray([0x01, 0x26, 0x00])


class Controlword(Enum):
    Power_on = 0x0F
    Power_off = 0x06
    Quick_stop = 0x0B
    Enable = 0x2F
    Start_abs_pos_1 = 0x3F
    Start_rel_pos_0 = 0x4F
    Start_rel_pos_1 = 0x5F
    Home_pos = 0x1F  # TODO: Check 0x0F 0x1F
    Clear = 0x80
    Zero = 0x00


class CMD(Enum):
    one_byte = 0x2F
    two_byte = 0x2B
    four_byte = 0x23


class DIN_Sim(Enum):
    d_1 = 0x01
    d_2 = 0x02
    d_3 = 0x04
    d_4 = 0x08
    d_5 = 0x10
    d_6 = 0x20
    d_7 = 0x40


class OperationMode(Enum):
    Pos_control = 1
    Vel_control = 3
    Torque_control = 4
    Vel_control_quick = -3
    Pulse_train_control = -4
    Home_mode = 6
    Diff_compl = 7


class Statusword:
    def __init__(self, data):
        value = int.from_bytes(data[5:7], byteorder="little", signed=True)
        value = [int(digit) for digit in bin(value)[2:]]
        while len(value) < 16:
            value.insert(0, 0)
        self.ready_on = value[15]
        self.switch_on = value[14]
        self.operation_enabled = value[13]
        self.fault = value[12]
        self.voltage_enabled = value[11]
        self.quick_stop = value[10]
        self.switch_on_disabled = value[9]
        self.warning = value[7]
        self.remote = value[6]
        self.target_reached = value[5]
        self.internal_limit_active = value[4]
        self.setpoint_acknowledge = value[3]
        self.following_error = value[2]
        self.communication_found = value[1]
        self.reference_switch = value[0]
        self.print()

    def print(self):
        print(
            f"      Ready on {self.ready_on}  Switch on {self.switch_on}  Operation enabled {self.operation_enabled}  Fault {self.fault}  Voltage enabled {self.voltage_enabled}  Quick stop {self.quick_stop}  Switch on disabled {self.switch_on_disabled}  Warning {self.warning}  Remote {self.remote}  Target reached {self.target_reached}  Internal limit active {self.internal_limit_active}  Setpoint acknowledge {self.setpoint_acknowledge}  Following error {self.following_error}  Communication found {self.communication_found}  Reference switch {self.reference_switch}"
        )


class PositionActualValue:
    def __init__(self, data):
        value = int.from_bytes(data[5:9], byteorder="little", signed=True)
        self.value = value
        self.print()

    def print(self):
        print(f"      Position actual value {self.value}")


class RS232Telegram:
    def __init__(self, id=None, data=None):
        if id:
            self._id = id
            self._data = bytearray(10)
            self._data[0] = id
        elif data:
            self._data = data
            self._print()
        else:
            raise ValueError("id or data must be provided")

    def _set_control_word(self, control_word: int):
        self._data[1] = 0x2B
        self._data[2:5] = Adress.Controlword.value
        self._data[5:9] = control_word.to_bytes(4, byteorder="little", signed=True)
        self._data[9] = self.calc_checksum(self._data)
        self._print("sending control word")
        return self._data

    def _set_operation_mode(self, operation_mode: int):
        self._data[1] = 0x2F
        self._data[2:5] = Adress.OperationMode.value
        self._data[5:9] = operation_mode.to_bytes(4, byteorder="little", signed=True)
        self._data[9] = self.calc_checksum(self._data)
        self._print("sending operation mode")
        return self._data

    def _set_target_position(self, target_pos: int):
        self._data[1] = 0x23
        self._data[2] = 0x7A
        self._data[3] = 0x60
        self._data[4] = 0x00
        self._data[5:9] = target_pos.to_bytes(4, byteorder="little", signed=True)
        self._data[9] = self.calc_checksum(self._data)
        self._print("sending operation mode")
        return self._data

    def _set_profile_speed(self, speed: int):
        self._data[1] = 0x23
        self._data[2] = 0x81
        self._data[3] = 0x60
        self._data[4] = 0x00
        _speed = int(speed * 512 * ENCODER_RESOLUTION / 1875)
        self._data[5:9] = _speed.to_bytes(4, byteorder="little", signed=True)
        self._data[9] = self.calc_checksum(self._data)
        self._print("sending profile speed")
        return self._data

    def _set_target_speed(self, speed: int):
        self._data[1] = 0x23
        self._data[2] = 0xFF
        self._data[3] = 0x60
        self._data[4] = 0x00
        _speed = int(speed * 512 * ENCODER_RESOLUTION / 1875)
        self._data[5:9] = _speed.to_bytes(4, byteorder="little", signed=True)
        self._data[9] = self.calc_checksum(self._data)
        self._print("sending profile speed")
        return self._data

    def _read_status_word(self):
        self._data[1] = 0x40
        self._data[2:5] = Adress.Statusword.value
        self._data[5:9] = 0x00.to_bytes(4, byteorder="little", signed=True)
        self._data[9] = self.calc_checksum(self._data)
        self._print("Reading status word")
        return self._data

    def _read_pos(self):
        self._data[1] = 0x40
        self._data[2:5] = Adress.PosActual.value
        self._data[5:9] = 0x00.to_bytes(4, byteorder="little", signed=True)
        self._data[9] = self.calc_checksum(self._data)
        self._print("Reading status word")
        return self._data

    def _read_data(self, register):
        self._data[1] = 0x40
        self._data[2:5] = register
        self._data[5:9] = 0x00.to_bytes(4, byteorder="little", signed=True)
        self._data[9] = self.calc_checksum(self._data)
        self._print("Reading status word")
        return self._data

    def _read_din_state(self):
        self._data[1] = 0x40
        self._data[2] = 0x10
        self._data[3] = 0x20
        self._data[4] = 0x0A
        self._data[5:9] = 0x00.to_bytes(4, byteorder="little", signed=True)
        self._data[9] = self.calc_checksum(self._data)
        self._print("Reading din state")
        return self._data

    def _test_read(self, data):
        if data[1] == 0x43:
            print(
                f"Readed data 4 bytes {[int(digit) for digit in bin(int.from_bytes(data[5:9], byteorder='little', signed=True))[2:]]}"
            )
            return True
        elif data[1] == 0x4B:
            print(
                f"Readed data 2 bytes {[int(digit) for digit in bin(int.from_bytes(data[5:7], byteorder='little', signed=True))[2:]]} "
            )
            return True
        elif data[1] == 0x4F:
            print(
                f"Readed data 1 bytes {[int(digit) for digit in bin(int.from_bytes(data[5:6], byteorder='little', signed=True))[2:]]} "
            )
            return True
        else:
            print("Not Readed data")

            return False

    def _test_write(self, data):
        if data[1] == 0x60:
            print("Writed data")
            return True
        elif data[1] == 0x80:
            print("Not Writed data")
            return False

    def _din_simulate(self, din_num):
        self._data[1] = CMD.two_byte.value
        self._data[2] = 0x10
        self._data[3] = 0x20
        self._data[4] = 0x02
        self._data[5:9] = din_num.to_bytes(4, byteorder="little", signed=True)
        self._data[9] = self.calc_checksum(self._data)
        self._print("Writing data u16 ")
        return self._data

    def _write_u16(self, register, value):
        self._data[1] = CMD.two_byte.value
        self._data[2] = register[0]
        self._data[3] = register[1]
        self._data[4] = register[2]
        self._data[5:9] = value.to_bytes(4, byteorder="little", signed=True)
        self._data[9] = self.calc_checksum(self._data)
        self._print("Writing data u16 ")
        return self._data

    @staticmethod
    def calc_checksum(telegram):
        checksum = -sum(telegram[:9]) % 256
        return checksum

    def _print(self, text=None):
        text = "" if text is None else text
        try:
            print(
                f"{text}  ID:{self._data[0]}  CMD: {hex(self._data[1])}  Index:{hex(self._data[2])} {hex(self._data[3])},  Subindex:{self._data[4]},  self._data:{self._data[5:9].hex()}  checksum: {hex(self._data[9])}"
            )
        except:
            pass
