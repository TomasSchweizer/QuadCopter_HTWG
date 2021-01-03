import usb.core
import usb.backend.libusb1
import usb.util
import struct
import numpy as np

# global variables
rx_transmission_counter = 0


# change if more or less bytes are send over USB
SENT_BYTES_INT16_RAW = 19
SENT_BYTES_FLOAT_RAW = 37
SENT_BYTES_FLOAT_FUSED_ANGLES = 13
SENT_BYTES_FLOAT_QUATERNIONS = 17


class USBBulkDevice:

    def __init__(self, id_vendor, id_product, USB_DATA):

        self.id_vendor = id_vendor
        self.id_product = id_product
        self.backend = usb.backend.libusb1.get_backend(find_library="Quadcopter_3D_Attitude/libusb-1.0.dll")
        self.dev = usb.core.find(idVendor=self.id_vendor, idProduct=self.id_product, backend=self.backend)
        # check if device was found
        try:
            print('Search for Device ...')
            while self.dev is None:
                self.dev = usb.core.find(idVendor=self.id_vendor, idProduct=self.id_product)

            if self.dev is not None:
                print('Device connected!')
        except usb.core.USBError as e:
            pass

        # set configuration to defaults (first found)
        self.dev.set_configuration()

        self.cfg = self.dev.get_active_configuration()
        self.intf = self.cfg[(0, 0)]
        self.ep_out = usb.util.find_descriptor(
            self.intf,
            # match the first OUT endpoint
            custom_match= lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)
        self.ep_in = ep_in = usb.util.find_descriptor(
            self.intf,
            # match the first IN endpoint
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)

        assert self.ep_out is not None
        assert self.ep_in is not None

        self.USB_DATA = USB_DATA

        print(self.ep_in)

    def usb_read_data(self):

        global rx_transmission_counter

        while True:

            try:
                data_in = self.ep_in.read(self.ep_in.wMaxPacketSize)

            except usb.core.USBError as e:
                data_in = None

            if data_in is None:
                pass

            else:

                if self.USB_DATA is 1:

                    if data_in[0] == 1:
                        relevant_data = data_in[1:SENT_BYTES_INT16_RAW]
                        convert_data = struct.unpack('<hhhhhhhhh', relevant_data)

                    elif data_in[0] == 2:
                        relevant_data = data_in[1:SENT_BYTES_FLOAT_RAW]
                        convert_data = struct.unpack('<fffffffff', relevant_data)
                    else:
                        print('Wrong data type')

                    x_acc = convert_data[0]
                    y_acc = convert_data[1]
                    z_acc = convert_data[2]

                    x_gyro = convert_data[3]
                    y_gyro = convert_data[4]
                    z_gyro = convert_data[5]

                    x_mag = convert_data[6]
                    y_mag = convert_data[7]
                    z_mag = convert_data[8]

                    rx_transmission_counter += 1

                    return x_acc, y_acc, z_acc, x_gyro, y_gyro, z_gyro, x_mag, y_mag, z_mag

                elif self.USB_DATA is 2:

                    if data_in[0] == 2:
                        relevant_data = data_in[1:SENT_BYTES_FLOAT_FUSED_ANGLES]
                        convert_data = struct.unpack('<fff', relevant_data)

                    else:
                        print('Wrong data type')

                    fused_angle_roll = convert_data[0]
                    fused_angle_pi = convert_data[1]
                    fused_angle_yaw = convert_data[2]

                    return fused_angle_roll, fused_angle_pi, fused_angle_yaw

                elif self.USB_DATA is 3:

                    if data_in[0] == 2:
                        relevant_data = data_in[1:SENT_BYTES_FLOAT_QUATERNIONS]
                        convert_data = struct.unpack('<ffff', relevant_data)
                    else:
                        print('Wrong data type')

                    quat_w = convert_data[0]
                    quat_x = convert_data[1]
                    quat_y = convert_data[2]
                    quat_z = convert_data[3]

                    quat = np.array([quat_w, quat_x, quat_y, quat_z])

                    return quat
                else:
                    print('Set USB_DATA in main.')

