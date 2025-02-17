#! /usr/bin/python3
import tkinter as tk
from tkinter import ttk
import requests
import json
import math
import time
import serial, serial.tools.list_ports
import simplebgc.commands, simplebgc.command_ids
import simplebgc.command_parser, simplebgc.serial, simplebgc.units
import struct
import traceback

# Function to calculate focal length for a given field of view
def calc_focal_length(fov):
    # Convert field of view in degrees to focal length in mm for a 35mm camera
    return 35 / (2 * math.tan(math.radians(fov / 2))) if fov > 0 else None

# In the following few functions we use the following coordinate system for local vectors:
# X points true east, Y true north, Z up
# Azimuth is degrees CW from true north
# Altitude degrees up from horizon
# Roll degrees CW from level
def get_alt_az_angles(vec):
    # TODO: set az=0 close to zenith
    az = (math.degrees(math.atan2(vec[0], vec[1])) + 360) % 360  # Normalize to 0-360 degrees
    alt = math.degrees(math.asin(vec[2]))
    return alt, az

def get_alt_az_vector(altaz):
    alt = math.radians(altaz[0])
    az = math.radians(altaz[1])
    return (math.sin(az) * math.cos(alt), math.cos(az) * math.cos(alt), math.sin(alt))

# Rotate about X axis
def rotate_x(vec, deg):
    a = math.radians(deg)
    cos = math.cos(a)
    sin = math.sin(a)
    return vec[0], cos * vec[1] - sin * vec[2], cos * vec[2] + sin * vec[1]

# Rotate about Y axis
def rotate_y(vec, deg):
    a = math.radians(deg)
    cos = math.cos(a)
    sin = math.sin(a)
    return cos * vec[0] + sin * vec[2], vec[1], cos * vec[2] - sin * vec[0]

# Rotate about Z axis
def rotate_z(vec, deg):
    a = math.radians(deg)
    cos = math.cos(a)
    sin = math.sin(a)
    return cos * vec[0] + sin * vec[1], cos * vec[1] - sin * vec[0], vec[2]

# Rotate by the negatives of the alt/az angles (X, Z axes) given as a vector
def rotate_altaz00(vec, altaz):
    # TODO: Normalize altaz
    altaz_xy = math.hypot(altaz[0], altaz[1])
    # Rotate to 0 azimuth
    az0 = (vec[0] * altaz[1] - vec[1] * altaz[0], vec[1] * altaz[1] + vec[0] * altaz[0], vec[2])
    # Rotate to 0 altitude
    return (az0[0], az0[1] * altaz_xy + az0[2] * altaz[2], az0[2] * altaz_xy - az0[1] * altaz[2])

def recalc_local_view_vector(t0_local_vec, t0_local_roll, local_lat, delta_t):
    delta_angle = delta_t * 360 / (1000000000 * 86164.0905) # delta_t (in ns) x 360 deg per 23h56min (in ns)

    # Our roll vector points to wherever left is in the camera frame.
    # Note that instead of applying t0_local_roll angle to the zero roll vector we transform our zero roll vector
    # as is from t0 to t1 time.  From it we get roll delta and we just add it to the t0 roll to obtain
    # t1 roll, this saves us some rotations.
    t0_local_zero_roll_vec = (-t0_local_vec[1], t0_local_vec[0], 0)

    # We're doing three rotations here
    # Instead we could calculate the north vector at current latitude and rotate the view and roll vecs around it,
    # this could be simpler although currently we avoid rotations around axes other than X and Y so that's a bonus.
    t0_eq_vec = rotate_x(t0_local_vec, -local_lat)
    t0_eq_zero_roll_vec = rotate_x(t0_local_zero_roll_vec, -local_lat)

    # The Earth has rotated by delta_angle so the sky has rotated -delta_angle for us
    t1_eq_vec = rotate_y(t0_eq_vec, -delta_angle)
    t1_eq_zero_roll_vec = rotate_y(t0_eq_zero_roll_vec, -delta_angle)

    t1_local_vec = rotate_x(t1_eq_vec, local_lat)
    t1_local_zero_roll_vec = rotate_x(t1_eq_zero_roll_vec, local_lat)

    # Rotate the roll vec to remove the view vector "component" (rotate to 0, 0 altitude and azimuth).
    # The easier alternative is to use only atan(roll.z / cos(view.z)) but this has tricky special cases.
    roll_vec = rotate_altaz00(t1_local_zero_roll_vec, t1_local_vec) # Y should be 0 except for FP errors
    roll_delta = math.degrees(math.atan2(roll_vec[2], -roll_vec[0])) # atan2 returns angle from Y=1 axis

    return t1_local_vec, t0_local_roll + roll_delta

class CalibrationData():
    def __init__(self):
        # Alt and Az offsets defined as the angle in degrees we need to add to the real sky Alt-Az
        # to get the gimbal to point at the right place.  This is just a starting point, we really
        # eventually need 2-point and 3-point calibration.  But since the gimbal maintains its own
        # sensor-based level/zenith reference, the pointing should be already good with only
        # azimuth calibration needed.  We'll also need vertical/horizontal offsets for the camera
        # mounting errors in the two axes (roll we don't care as much...), which we don't have yet,
        # and the roll-center offsets which can maybe be fused with the above mounting error
        # offsets.
        self.az_offset = 0.0
        self.alt_offset = 0.0        # Not used but may be informative
        self.roll_offset_right = 0.0 # Not currently used
        self.roll_offset_up = 0.0    # Not currently used

    def apply_forward(self, vec, roll):
        return rotate_z(vec, self.az_offset), roll

    def apply_reverse(self, vec, roll):
        return rotate_z(vec, -self.az_offset), roll

    def updated(self):
        print(f"Calibration updated to ({self.az_offset:.3f}°, {self.alt_offset:.3f}°, " + \
            f"{self.roll_offset_right:.3f}°, {self.roll_offset_up:.3f}°)")

# The below is now unused, needs to be adapted to work with the vector representatoin instead of
# with the angles.
def apply_roll_offset_to_altaz(altaz, roll, offset):
    rroll = math.radians(roll)
    one_minus_cos = 1.0 - math.cos(rroll)
    sin = math.sin(rroll)

    # Apply a correction for camera lens forward axis being slightly off from the gimbal roll axis.
    #
    # This is only an approximation because the constant-altitude line in alt-az coordinates forms a cone
    # surface in 3D space while the constant Y line in the image frame (horizontal line) forms a plane in
    # 3D, yet we calculate the horizontal offset as azimuth offset.  Additionally changing azimuth causes
    # a roll in the image.
    # But this approximation should be good for small offset angles, especially at lower altitudes.
    # So we give up on applying the azimuth correction if we're very to zenith, otherwise we apply
    # a factor of 1 / cos(altitude) to account for the same amount of shift in image coordinates causing
    # a bigger difference in azimuth angle as altitude goes up.
    #
    # Ideally we should be applying this whole roll correction when we deal with vectors rather than
    # angles, perhaps in recalc_local_view_vector.
    alt_offset = -(one_minus_cos * offset[0] + sin * offset[1])
    az_offset = -(one_minus_cos * offset[1] - sin * offset[0])

    if altaz[0] < 89.0:
        az_offset /= math.cos(math.radians(altaz[0]))
        if math.abs(az_offset) > math.radians(1):
            az_offset = 0
    else:
        az_offset = 0

    return (altaz[0] + alt_offset, altaz[1] + az_offset)

# The following 4 functions return/accept angles in the order Yaw, Pitch, Roll (ypr),
# regardless of the gimbal/camera configuration.  This is not the same as SimpleBGC API
# standard order which uses Roll, Pitch, Yaw everywhere.

# The Pilotfly H2 "euler order" is Cam, Pitch, Roll, Yaw, i.e. the pitch motor+arm is mounted
# at the end of the roll motor+arm.  However by mounting the camera "sideways", looking
# "right" instead of "forward" wrt the usual reference frame, the camera pitch and roll axes
# are switched around with regards to gimbal pitch and roll.  Yaw remains yaw.
# The angles now match azimuth, altitude and roll without new math operations other than
# sign changes and we still get a full range of camera motion for pointing at celestial
# coordinates and rolling to any angle.
#
# SimpleBGC 2.6 Serial Protocol Specification, Appendix D: Coordinate system conversions,
# Euler angles:
# "Serial API uses the following convention for Euler angles:
#  • ROLL is over Y axis (forward), clockwise positive (if looking towards the axis direction)
#  • PITCH is over X axis (right), counter-clockwise positive
#  • YAW is over Z axis (down), clockwise positive"
#
# Use the following two functions if your camera is mounted this way.  The math is definitely
# simpler.
def get_simplebgc_ypr_from_vec(vec, roll):
    altaz = get_alt_az_angles(vec)
    return altaz[1], -roll, -altaz[0]

def get_vec_from_simplebgc_ypr(g_yaw, g_pitch, g_roll):
    return get_alt_az_vector((-g_roll, g_yaw)), -g_pitch

# Use the following for standard camera position on Pilotfly H2 and most other SimpleBGC
# gimbals.  Note the names are slightly different from the above ("yrp" vs. "ypr".)
def get_simplebgc_yrp_from_vec(vec, roll):
    # TODO: Normalize vec

    cos_alt = vec[2]
    sin_alt = math.hypot(vec[0], vec[1])
    cos_az = vec[1] / sin_alt
    sin_az = vec[0] / sin_alt
    rroll = math.radians(roll)
    cos_roll = math.cos(rroll)
    sin_roll = math.sin(rroll)

    # Get the camera "right" vector in the local reference frame (northing, easting, alt),
    # this is going to be the desired gimbal pitch axis.
    alt0az0_right = (cos_roll, 0, -sin_roll) # Point right in the camera frame
    az0_right = (alt0az0_right[0], -sin_alt * alt0az0_right[2], cos_alt * alt0az0_right[2])
    right = (cos_az * az0_right[0] + sin_az * az0_right[1], cos_az * az0_right[1] - sin_az * az0_right[0], az0_right[2])

    right_cast_len = math.hypot(right[0], right[1])
    if right_cast_len > 0.0001:
        cos_g_yaw = right[0] / right_cast_len
        sin_g_yaw = -right[1] / right_cast_len
    else:
        # If yaw and pitch axes are basically aligned, we only care about the sum of yaw+pitch
        # Arbitrarily set yaw to 0 and pitch based on vec
        cos_g_yaw = 1.0
        sin_g_yaw = 0.0
    cos_g_roll = cos_g_yaw * right[0] - sin_g_yaw * right[1] # Should be same as right_cast_len but signed
    sin_g_roll = -right[2]
    yaw0_vec = (cos_g_yaw * vec[0] - sin_g_yaw * vec[1], cos_g_yaw * vec[1] + sin_g_yaw * vec[0], vec[2])
    cos_g_pitch = yaw0_vec[1]
    sin_g_pitch = cos_g_roll * yaw0_vec[2] + sin_g_roll * yaw0_vec[0] # Same as math.hypot(yaw0_vec[0], yaw0_vec[2]) but signed

    return (math.degrees(math.atan2(sin_g_yaw, cos_g_yaw)),
        math.degrees(-math.atan2(sin_g_pitch, cos_g_pitch)),
        math.degrees(math.atan2(sin_g_roll, cos_g_roll)))

def get_vec_from_simplebgc_yrp(g_yaw, g_pitch, g_roll):
    yaw0roll0_vec = rotate_x((0, 1, 0), -g_pitch)
    yaw0_vec = rotate_y(yaw0roll0_vec, g_roll)
    yaw0_right = rotate_y((1, 0, 0), g_roll)
    vec = rotate_z(yaw0_vec, g_yaw)

    yaw0pitch0_right = rotate_altaz00(yaw0_right, yaw0_vec)
    roll = math.degrees(math.atan2(-yaw0pitch0_right[2], yaw0pitch0_right[0]))
    return vec, roll

# Function to fetch data from Stellarium API
def fetch_stellarium_view_data():
    # TODO: make async, plug into the main loop
    response = requests.get("http://localhost:8090/api/main/view")
    response.raise_for_status()
    data = response.json()
    s_x, s_y, z = json.loads(data.get("altAz"))
    # Stellarium altAz is defined as:
    # "the actually returned values are based on an azimuth Az' counted from South (x=1) towards East (y=1), Az'=180-Az."
    # so just update x and y:
    x = s_y # Easting
    y = -s_x # Northing

    response = requests.get("http://localhost:8090/api/main/status")
    response.raise_for_status()
    data = response.json()
    fov = data.get("view").get("fov")
    lat = data.get("location").get("latitude")
    lon = data.get("location").get("longitude")

    return (x, y, z), fov, (lat, lon)

def log_error(msg, ex):
    print(f'{msg}: {ex} ({type(ex).__name__}) at')
    traceback.print_tb(ex.__traceback__)

class GimbalConnection:
    def __init__(self, error_cb):
        self.port_path = None
        self.port = None
        self.query_cb = None
        self.error_cb = error_cb
        self.buf = bytes()
        self.query_type = 0
        self.mainloop = None

    def connect(self, path, mainloop):
        if self.port is not None:
            raise Exception("Already connected")

        self.port = serial.Serial(path, baudrate=115200, timeout=0, write_timeout=1)
        self.port_path = path

        mainloop.createfilehandler(self.port, tk.READABLE, self.recv)
        self.mainloop = mainloop

    def disconnect(self):
        if self.port is None:
            raise Exception("Already disconnected")

        self.mainloop.deletefilehandler(self.port)
        self.port.close()
        self.port = None
        self.port_path = None
        self.buf = bytes()

        if self.query_cb is not None:
            self.reply_query(-1)

    def connected_path(self):
        return self.port_path

    def error(self, err):
        self.disconnect()
        self.error_cb(err)

    def reply_query(self, status, **kwargs):
        cb = self.query_cb
        self.query_cb = None
        cb(status, **kwargs)

    def handle_cmd(self, raw_cmd):
        name = simplebgc.command_ids.get_command_name(raw_cmd.id)
        print('Received ' + name + ' from gimbal')

        try:
            cmd = simplebgc.command_parser.parse_cmd(raw_cmd)
        except Exception as e:
            log_error('Parse error', e)
            # self.error(e)
            return

        if isinstance(cmd, simplebgc.commands.ConfirmInCmd):
            cmd_name = simplebgc.command_ids.get_command_name(cmd.cmd_id)
            print(f'{cmd_name} confirmed, data {cmd.data}')
            return

        if isinstance(cmd, simplebgc.commands.ErrorInCmd):
            cmd_name = simplebgc.command_ids.get_command_name(cmd.cmd_id)
            try:
                err_name = simplebgc.commands.ErrorCodeBasic(cmd.error_code).name
            except:
                err_name = f"UNKNOWN({cmd.error_code})"
            print(f'{cmd_name} returned error {err_name} data {cmd.error_data}')
            return

        if isinstance(cmd, simplebgc.commands.EventInCmd):
            try:
                evt_id_str = simplebgc.commands.EventID(cmd.event_id).name
            except:
                evt_id_str = f"UNKNOWN({cmd.event_id})"
            print(f'Gimbal event {evt_id_str} type {cmd.event_type} param1 {cmd.param1}')
            return

        if isinstance(cmd, simplebgc.commands.RealtimeData3InCmd):
            if self.query_cb and self.query_type == 1:
                self.reply_query(0, on=(cmd.rt_data_flags & 1 == 1))

            return

        if isinstance(cmd, simplebgc.commands.RealtimeDataCustomInCmd):
            if self.query_cb and self.query_type == 2:
                if len(cmd.data) == 0:
                    self.error(Exception('No data, firmware < 2.70b6? Should switch to IMU_ANGLES or IMU_ANGLES_RAD'))
                    return
                try:
                    raw_roll, raw_pitch, raw_yaw = struct.unpack('<iii', cmd.data)
                except Exception as e:
                    log_error('RealtimeDataCustomInCmd parse error', e)
                    self.error(e)
                    return

                yaw = simplebgc.units.to_degree_hires(raw_yaw)
                pitch = simplebgc.units.to_degree_hires(raw_pitch)
                roll = simplebgc.units.to_degree_hires(raw_roll)

                self.reply_query(0, angles=(yaw, pitch, roll))

            return

    def recv(self, port, mark):
        num = 256
        try:
            buf = self.port.read(num)
        except Exception as e:
            self.error(e)
            return

        self.buf += buf

        while len(buf) >= 5: # Header + v1 crc
            try:
                msg, consumed = simplebgc.serial.unpack_message(self.buf)
            except simplebgc.serial.DataShortException as e:
                break
            except Exception as e:
                # TODO: Actually spec says we need to re-sync by looking for a new start-char
                self.error(e)
                return

            self.buf = self.buf[consumed:]
            self.handle_cmd(simplebgc.serial.RawCmd(id=msg.command_id, payload=msg.payload))

    def send_cmd(self, cmd):
        self.port.write(simplebgc.serial.pack_message(cmd))

    def get_motors_on_off(self, cb):
        if self.query_cb is not None:
            raise Exception("Already querying")

        self.send_cmd(simplebgc.serial.create_message(simplebgc.command_ids.CMD_REALTIME_DATA_3))

        self.query_cb = cb
        self.query_type = 1

    def set_motors_on_off(self, on):
        if on:
            self.send_cmd(simplebgc.serial.create_message(simplebgc.command_ids.CMD_MOTORS_ON))
        else:
            payload = simplebgc.commands.MotorsOffOutCmd(mode=1) # Brake mode (high impedance)
            self.send_cmd(simplebgc.serial.create_message(simplebgc.command_ids.CMD_MOTORS_OFF, payload.pack()))

    def get_angles(self, cb):
        if self.query_cb is not None:
            raise Exception("Already querying")

        # Request the 20bit fixed-point format
        # Alternatively CMD_REALTIME_DATA_CUSTOM can request 24-bit encoder values, could be interesting
        # Also alternatively CMD_EXT_MOTORS_STATE can request 32-bit angles and speeds but this likely
        # reports the frame-to-main angles, not absolute IMU angles.  Does it support the 3 main motors?
        payload = simplebgc.commands.RealtimeDataCustomOutCmd(flags=1 << 17) # IMU_ANGLES_20
        self.send_cmd(simplebgc.serial.create_message(simplebgc.command_ids.CMD_REALTIME_DATA_CUSTOM, payload.pack()))

        self.query_cb = cb
        self.query_type = 2

    def set_angles(self, yaw, pitch, roll, smooth_brake=True):
        mode = \
            simplebgc.commands.ControlMode.MODE_ANGLE | \
            simplebgc.commands.ControlMode.CONTROL_FLAG_HIGH_RES_SPEED | \
            ((0 if smooth_brake else 1) << 5) # CONTROL_FLAG_TARGET_PRECISE
        ### TODO: convert to flags

        payload = simplebgc.commands.ControlExtOutCmd(
            data_set=int("001100011000110", 2), # 4-byte fixed-point angles for all axes
            yaw_mode=mode,
            pitch_mode=mode,
            roll_mode=mode,
            yaw_angle=simplebgc.units.from_degree_hires(yaw),
            pitch_angle=simplebgc.units.from_degree_hires(pitch),
            roll_angle=simplebgc.units.from_degree_hires(roll))

        self.send_cmd(simplebgc.serial.create_message(simplebgc.command_ids.CMD_CONTROL_EXT, payload.pack()))

    def set_speeds_and_angles(self, yaw, pitch, roll, yaw_rate, pitch_rate, roll_rate, smooth_brake=True):
        mode = \
            simplebgc.commands.ControlMode.MODE_SPEED_ANGLE | \
            simplebgc.commands.ControlMode.CONTROL_FLAG_HIGH_RES_SPEED | \
            ((0 if smooth_brake else 1) << 5) # CONTROL_FLAG_TARGET_PRECISE

        payload = simplebgc.commands.ControlExtOutCmd(
            data_set=int("011110111101111", 2), # 4-byte fixed-point speeds and angles for all axes
            yaw_mode=mode,
            pitch_mode=mode,
            roll_mode=mode,
            yaw_speed=simplebgc.units.from_degree_per_sec_hires(yaw_rate),
            pitch_speed=simplebgc.units.from_degree_per_sec_hires(pitch_rate),
            roll_speed=simplebgc.units.from_degree_per_sec_hires(roll_rate),
            yaw_angle=simplebgc.units.from_degree_hires(yaw),
            pitch_angle=simplebgc.units.from_degree_hires(pitch),
            roll_angle=simplebgc.units.from_degree_hires(roll))

        self.send_cmd(simplebgc.serial.create_message(simplebgc.command_ids.CMD_CONTROL_EXT, payload.pack()))

    def is_busy(self):
        return self.query_cb is not None

class GimbalGoToApp:
    default_port = "/dev/rfcomm0"
    # Whether gimbal angles map to camera ypr or yrp
    ypr = False

    def __init__(self, window):
        self.window = window
        self.window.title("Gimbal GoTo")

        # Bind Escape key to exit
        self.window.bind("<Escape>", lambda event: self.exit_application())
        # TODO: bind arrow keys

        # Variables to hold the info displayed in the UI
        self.azimuth = tk.DoubleVar(value=0.0)
        self.altitude = tk.DoubleVar(value=0.0)
        self.roll = tk.DoubleVar(value=0.0)
        self.observer_latitude = tk.DoubleVar(value=0.0)
        self.fov_str = tk.StringVar(value="Unknown")
        self.focal_length_str = tk.StringVar(value="Unknown")
        self.error_message = tk.StringVar(value="")
        self.sel_port_path = tk.StringVar()
        self.conn_port_str = tk.StringVar(value="Disconnected")

        # TODO: convert to sliders, automatically update self.calibaration on
        # changes
        self.roll_offsets = (tk.DoubleVar(value=0.0), tk.DoubleVar(value=0.0))

        # Toggle state
        self.tracking_disabled = tk.BooleanVar(value=True)
        self.motors_off = tk.BooleanVar(value=False)
        self.gimbal_paused = tk.BooleanVar(value=True)

        # Create GUI widgets
        self.create_widgets()

        # Internal state
        self.gimbal = GimbalConnection(self.port_error_cb)
        self.tracking_timer = None
        self.base_view_vector = None
        self.base_data_timestamp = None # Time the tracking was started, data was fetched or manually adjusted
        self.observer_ll = None
        self.fov = None
        self.calibration = CalibrationData()

        self.update_widget_states()
        # TODO: refresh automatically, how? maybe listen to udev events, also automatically recognize USB serial as
        # well as listen to bluetoothctl to automatically connect to known gimbal names (Pilotfly) or devices with
        # the BT serial port profile, in either case only when previously paired.  Also automatically run
        # "rfcomm connect <next-free-/dev/rfcommN> <bdaddr> <chan>"
        self.refresh_ports()

    def create_widgets(self):
        row = -1
        def next():
            nonlocal row
            row += 1
            return row

        # Container frame for centering
        frame = ttk.Frame(self.window)
        frame.grid(row=0, column=0, padx=10, pady=10, sticky="NSEW")

        az_row = next()
        alt_row = next()
        roll_row = next()
        lat_row = next()
        fov_row = next()
        fl_row = next()

        ttk.Label(frame, text="Azimuth (°):").grid(row=az_row, column=0, padx=5, pady=5, sticky="E")
        ttk.Label(frame, text="Altitude (°):").grid(row=alt_row, column=0, padx=5, pady=5, sticky="E")
        ttk.Label(frame, text="Roll offset (°):").grid(row=roll_row, column=0, padx=5, pady=5, sticky="E")
        ttk.Label(frame, text="Observer latitude (°):").grid(row=lat_row, column=0, padx=5, pady=5, sticky="E")
        ttk.Label(frame, text="Field of view (°):").grid(row=fov_row, column=0, padx=5, pady=5, sticky="E")
        ttk.Label(frame, text="Focal Length (mm):").grid(row=fl_row, column=0, padx=5, pady=5, sticky="E")

        ttk.Label(frame, textvariable=self.azimuth).grid(row=az_row, column=1, padx=5, pady=5, sticky="W")
        ttk.Label(frame, textvariable=self.altitude).grid(row=alt_row, column=1, padx=5, pady=5, sticky="W")
        ttk.Label(frame, textvariable=self.roll).grid(row=roll_row, column=1, padx=5, pady=5, sticky="W")
        ttk.Label(frame, textvariable=self.observer_latitude).grid(row=lat_row, column=1, padx=5, pady=5, sticky="W")
        ttk.Label(frame, textvariable=self.fov_str).grid(row=fov_row, column=1, padx=5, pady=5, sticky="W")
        ttk.Label(frame, textvariable=self.focal_length_str).grid(row=fl_row, column=1, padx=5, pady=5, sticky="W")

        # Error message display
        error_label = ttk.Label(frame, textvariable=self.error_message, foreground="red", wraplength=300)
        error_label.grid(row=next(), column=0, columnspan=2, pady=5)

        # Arrow buttons for manual adjustment
        button_frame = ttk.Frame(frame)
        button_frame.grid(row=next(), column=0, columnspan=2, pady=10)

        self.arrow_buttons = [
            ttk.Button(button_frame, text="↑", command=lambda: self.adjust_angles(0, 1)),
            ttk.Button(button_frame, text="↓", command=lambda: self.adjust_angles(0, -1)),
            ttk.Button(button_frame, text="←", command=lambda: self.adjust_angles(-1, 0)),
            ttk.Button(button_frame, text="→", command=lambda: self.adjust_angles(1, 0))
        ]
        self.arrow_buttons[0].grid(row=0, column=1)
        self.arrow_buttons[1].grid(row=2, column=1)
        self.arrow_buttons[2].grid(row=1, column=0)
        self.arrow_buttons[3].grid(row=1, column=2)
        # TODO roll buttons? zoom slider? also add sliders for roll centre calibration
        # (self.roll_offsets)
        # The idea for roll centre calibration is that the user sets very high zoom on the
        # camera, sets the same focal length value here (or rather in Stellarium, then
        # loads view from Stellarium), rotates the camera using roll buttons, observes
        # where the centre of rotation/pivot point is within the image frame (or rather,
        # WRT the image frame since the centre could still be outside) and sets the
        # sliders to X and Y of that point WRT to the frame centre with camera back to
        # level.  From there, any time we apply the roll here we also apply a correction
        # to alt/az to keep the centre close to the actual centre.

        # Checkboxes
        self.tracking_button = ttk.Checkbutton(
            frame, text="Pause sky tracking", variable=self.tracking_disabled, command=self.toggle_tracking
        )
        self.tracking_button.grid(row=next(), column=1, columnspan=1, pady=1, sticky="W")

        self.motors_button = ttk.Checkbutton(
            frame, text="Motors off", variable=self.motors_off, command=self.toggle_motors
        )
        self.motors_button.grid(row=next(), column=1, columnspan=1, pady=1, sticky="W")

        self.updates_button = ttk.Checkbutton(
            frame, text="Pause gimbal updates", variable=self.gimbal_paused, command=self.update_widget_states
        )
        self.updates_button.grid(row=next(), column=1, columnspan=1, pady=1, sticky="W")

        # Buttons
        s_row = next()
        g_row = next()
        button = ttk.Button(frame, text="Fetch from Stellarium view", command=self.fetch_stellarium)
        button.grid(row=s_row, column=0, columnspan=1, pady=5)
        self.fetch_gimbal_button = ttk.Button(frame, text="Fetch from gimbal", command=self.fetch_gimbal)
        self.fetch_gimbal_button.grid(row=g_row, column=0, columnspan=1, pady=5)
        ### To Stellarium view
        ### Fetch from Stellarium object
        ### Set offset pt A ... 1 is only enough for a basic azimuth offset + altitude offset at one point on the circle or close to it
        ### Set offset pt B ... are 2 points enough? should be enough, there's exactly one great circle passing through 2 points
        self.calib_s_button = ttk.Button(frame, text="Fetch+Calibrate", command=self.calib_stellarium)
        self.calib_s_button.grid(row=s_row, column=1, columnspan=1, pady=5)
        self.calib_g_button = ttk.Button(frame, text="Fetch+Calibrate", command=self.calib_gimbal)
        self.calib_g_button.grid(row=g_row, column=1, columnspan=1, pady=5)

        ports_row = next()
        button = ttk.Button(frame, text="Refresh ports", command=self.refresh_ports)
        button.grid(row=ports_row, column=0, padx=5, pady=5, sticky="E")
        self.port_dropdown = ttk.Combobox(frame, textvariable=self.sel_port_path, state="readonly")
        self.port_dropdown.grid(row=ports_row, column=1, padx=5, pady=10, sticky="W")

        connect_row = next()
        self.conn_button = ttk.Button(frame, text="Connect", command=self.port_connect_disconnect)
        self.conn_button.grid(row=connect_row, column=0, padx=5, pady=5, sticky="E")
        ttk.Label(frame, textvariable=self.conn_port_str, wraplength=300).grid(row=connect_row, column=1, padx=5, pady=5, sticky="W")

    def set_widget_state(self, widget, active):
        widget.config(state="normal" if active else "disabled")

    def update_widget_states(self):
        connected = self.gimbal.connected_path()
        gimbal_updating = not self.tracking_disabled.get() and not self.motors_off.get() and not self.gimbal_paused.get()
        for w in self.arrow_buttons:
            self.set_widget_state(w, self.base_view_vector is not None)
        self.set_widget_state(self.tracking_button, self.base_view_vector is not None)
        self.set_widget_state(self.motors_button, connected and not self.gimbal.is_busy())
        self.set_widget_state(self.updates_button, connected)
        self.set_widget_state(self.calib_s_button, self.base_view_vector is not None)
        self.set_widget_state(self.fetch_gimbal_button, connected and not self.gimbal.is_busy() and not gimbal_updating)
        self.set_widget_state(self.calib_g_button, connected and not self.gimbal.is_busy() and
                         self.base_view_vector is not None and not gimbal_updating)
        self.set_widget_state(self.conn_button, (not connected and self.sel_port_path.get()) or not self.gimbal.is_busy())
        self.conn_button.config(text="Disconnect" if connected else "Connect")

    def refresh_ports(self):
        """Fetch and update the list of available serial ports."""
        sel = self.sel_port_path.get()
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_dropdown["values"] = ports
        if sel in ports:
            pass
        elif self.default_port in ports:
            self.sel_port_path.set(self.default_port)
        elif ports:
            self.port_dropdown.current(0) # Select the first port by default
        else:
            self.sel_port_path.set("")    # Clear the dropdown

    def port_error_cb(self, e):
        # Currently when this is called, port has been unconditionally closed
        self.conn_port_str.set(f"Communication error: {e}")
        log_error("Communication error", e)
        self.motors_off.set(False)
        self.update_widget_states()

    def port_connect_disconnect(self):
        if self.gimbal.connected_path() is not None:
            self.gimbal.disconnect()
            self.conn_port_str.set("Disconnected")
            print("Disconnected")
            self.motors_off.set(False)
        else:
            path = self.sel_port_path.get()
            try:
                self.gimbal.connect(path, self.window)
                self.gimbal.get_motors_on_off(self.motors_on_off_cb)
                self.conn_port_str.set("Connected to " + self.gimbal.connected_path())
                print("Connected to " + self.gimbal.connected_path())
            except Exception as e:
                self.conn_port_str.set(f"Error connecting to {path}: {e}")
                log_error(f"Error connecting to {path}", e)

        self.update_widget_states()

    def motors_on_off_cb(self, status, on=True):
        self.motors_off.set(not on)
        self.update_widget_states()

    def toggle_motors(self):
        if self.gimbal.connected_path() is None or self.gimbal.is_busy():
            raise Exception("Wrong gimbal connection state")

        self.gimbal.set_motors_on_off(not self.motors_off.get())
        self.update_widget_states()

    @classmethod
    def get_gimbal_angles_from_vec(cls, vec, roll):
        if cls.ypr:
            return get_simplebgc_ypr_from_vec(vec, roll)
        else:
            return get_simplebgc_yrp_from_vec(vec, roll)

    @classmethod
    def get_vec_from_gimbal_angles(cls, g_yaw, g_pitch, g_roll):
        if cls.ypr:
            return get_vec_from_simplebgc_ypr(g_yaw, g_pitch, g_roll)
        else:
            return get_vec_from_simplebgc_yrp(g_yaw, g_pitch, g_roll)

    def update_gimbal(self, continuous):
        if self.gimbal.connected_path() is None or self.gimbal_paused.get():
            return

        delta = self.get_delta()
        local_vec, roll = recalc_local_view_vector(self.base_view_vector, 0.0, self.observer_ll[0], delta)
        calib_vec, calib_roll = self.calibration.apply_forward(local_vec, roll)

        if not continuous:
            g_yaw, g_pitch, g_roll = self.get_gimbal_angles_from_vec(calib_vec, calib_roll)

            self.gimbal.set_angles(g_yaw, g_pitch, g_roll)

        if self.base_data_timestamp is None:
            return

        # Calculate approx speed/rate from the difference between some future position and now,
        # use a 5 second period (5 one-second update periods) to smooth the movement out.
        #
        # We want to set a speed/rate setpoint and not an angle setpoint here.  In theory using
        # MODE_SPEED_ANGLE partly frees us from having to request actual current angles from the
        # gimbal and updating the speed to correct drift from calculated expected current angles.
        # TODO: revise, should we drop the below? currently we set the current angles because whatever angles we set we'll be drifting towards them or away from
        #  them at some point so whatever we set should probably be the desired angles within the 0 to 1 second period from now.  If the drift is kept within that
        #  window we don't care. perhaps set to 0.5 sec from now? nah, shouldn't matter?
        # We also set the target angles form MODE_SPEED_ANGLE to those 5-seconds-into-the-future
        # angles instead of 1-second (expected next tracking update) to avoid a race between
        # our next timer callback and the gimbal's movement reaching target angles.
        #
        # TODO: we do need to do the angle tracking though because by controlling only the speed
        # we may be drifting towards or away from the target angles.
        dt = 5
        t1_delta = delta + dt * 1000000000
        t1_vec, t1_roll = recalc_local_view_vector(self.base_view_vector, 0.0, self.observer_ll[0], t1_delta)
        calib_t1_vec, calib_t1_roll = self.calibration.apply_forward(t1_vec, t1_roll)

        t0_g_angles = self.get_gimbal_angles_from_vec(calib_vec, calib_roll)
        t1_g_angles = self.get_gimbal_angles_from_vec(calib_t1_vec, calib_t1_roll)
        g_rates = (
            (t1_g_angles[0] - t0_g_angles[0]) / dt,
            (t1_g_angles[1] - t0_g_angles[1]) / dt,
            (t1_g_angles[2] - t0_g_angles[2]) / dt)

        self.gimbal.set_speeds_and_angles(*t0_g_angles, *g_rates,  smooth_brake=False)

    @staticmethod
    def get_timestamp():
        return time.clock_gettime_ns(time.CLOCK_BOOTTIME) # Or time.time_ns() for portability

    def get_delta(self):
        if self.base_data_timestamp is None:
            return 0

        return self.get_timestamp() - self.base_data_timestamp

    def set_view_vector(self, vec, update_gimbal=True):
        self.base_view_vector = vec
        if self.tracking_timer is not None:
            self.base_data_timestamp = self.get_timestamp()
        # Roll always resets to 0 at base_data_timestamp so not saved

        self.update_widget_states()

        # Set a new angle setpoint, not a rate setpoint and don't limit the rates to the sky tracking rates
        if update_gimbal:
            self.update_gimbal(continuous=False)

    def fetch_stellarium(self):
        try:
            vec, fov, obs = fetch_stellarium_view_data()
        except Exception as e:
            self.error_message.set(f"Error fetching or parsing data: {e}")
            log_error('Error fetching or parsing data', e)
            return

        self.set_view_vector(vec)
        self.observer_ll = obs
        self.fov = fov

        self.error_message.set("")  # Clear any previous error message
        self.update_ui_info()

    def calib_stellarium(self):
        try:
            vec, fov, obs = fetch_stellarium_view_data()
        except Exception as e:
            self.error_message.set(f"Error fetching or parsing data: {e}")
            log_error('Error fetching or parsing data', e)
            return

        # Not strictly raw gimbal vector as it has our offsets applied (if any) so we will add the
        # angle differences to existing calibration angles instead of replacing them.
        delta = self.get_delta()
        gimbal_off_vec, roll = recalc_local_view_vector(self.base_view_vector, 0.0, self.observer_ll[0], delta)

        true_altaz = get_alt_az_angles(vec)
        gimbal_off_altaz = get_alt_az_angles(gimbal_off_vec)

        self.calibration.az_offset += gimbal_off_altaz[1] - true_altaz[1]
        self.calibration.az_offset = self.calibration.az_offset % 360.0
        self.calibration.alt_offset = gimbal_off_altaz[0] - true_altaz[0]
        self.calibration.updated()

        self.set_view_vector(vec)
        self.observer_ll = obs
        self.fov = fov

        self.error_message.set("")  # Clear any previous error message
        self.update_ui_info()

    def fetch_gimbal(self):
        if self.gimbal.connected_path() is None or self.gimbal.is_busy():
            raise Exception("Wrong gimbal connection state")

        self.gimbal.get_angles(self.gimbal_angles_cb)
        self.update_widget_states()

    def gimbal_angles_cb(self, status, angles=(0.0, 0.0, 0.0)):
        if status != 0:
            self.update_widget_states()
            return

        calib_vec, calib_roll = self.get_vec_from_gimbal_angles(*angles)
        vec, roll = self.calibration.apply_reverse(calib_vec, calib_roll)
        self.set_view_vector(vec, update_gimbal=False)

        if self.observer_ll is None:
            self.observer_ll = (40.0, 0.0) # Accept as command line parameter instead? Do we maybe need config class?

        if self.fov is None:
            self.fov = 30.0

        self.update_widget_states()
        self.update_ui_info()

    def calib_gimbal(self):
        if self.gimbal.connected_path() is None or self.gimbal.is_busy():
            raise Exception("Wrong gimbal connection state")

        self.gimbal.get_angles(self.gimbal_calib_cb)
        self.update_widget_states()

    def gimbal_calib_cb(self, status, angles=(0.0, 0.0, 0.0)):
        if status != 0:
            self.update_widget_states()
            return

        delta = self.get_delta()
        local_vec, roll = recalc_local_view_vector(self.base_view_vector, 0.0, self.observer_ll[0], delta)

        gimbal_vec, gimbal_roll = self.get_vec_from_gimbal_angles(*angles)

        true_altaz = get_alt_az_angles(local_vec)
        gimbal_altaz = get_alt_az_angles(gimbal_vec)

        self.calibration.az_offset = (gimbal_altaz[1] - true_altaz[1]) % 360.0
        self.calibration.alt_offset = gimbal_altaz[0] - true_altaz[0]
        self.calibration.updated()

        self.update_widget_states()
        pass

    def update_ui_info(self):
        if self.base_view_vector is None:
            return

        try:
            local_vec, roll = recalc_local_view_vector(self.base_view_vector, 0.0, self.observer_ll[0], self.get_delta())
            altaz = get_alt_az_angles(local_vec)
            focal_length = calc_focal_length(self.fov)
            zoom = 81.2 / self.fov
            zoom = round(zoom, 1) if zoom < 1.8 else round(zoom)
        except Exception as e:
            self.error_message.set(f"Maths error: {e}")
            log_error('Maths error', e)
            return

        self.azimuth.set(round(altaz[1], 4))
        self.altitude.set(round(altaz[0], 4))
        self.roll.set(round(roll, 4))
        self.observer_latitude.set(round(self.observer_ll[0], 4))
        self.fov_str.set(f"{self.fov:.1f}° -- x{zoom:g} zoom")
        self.focal_length_str.set(f"{focal_length:.0f} mm")

    def adjust_angles(self, delta_azimuth, delta_altitude):
        if self.fov is None:
            return

        local_vec, roll = recalc_local_view_vector(self.base_view_vector, 0.0, self.observer_ll[0], self.get_delta())
        altaz = get_alt_az_angles(local_vec) # Could also directly rotate base_view_vector but more code

        # TODO: Should azimuth steps be scaled with altitude at low FOV?
        step = self.fov / 8
        altaz = (altaz[0] + delta_altitude * step, altaz[1] + delta_azimuth * step)

        self.set_view_vector(get_alt_az_vector(altaz))
        self.update_ui_info()

    def toggle_tracking(self):
        if self.tracking_disabled.get():
            print("Tracking disabled.")
            self.stop_tracking()
        else:
            print("Tracking enabled.")
            self.start_tracking()

        self.update_widget_states()

    def update_timer(self):
        self.tracking_timer = self.window.after(1000, self.update_tracking)  # Schedule next update

    def start_tracking(self):
        if self.tracking_timer is not None:
            return

        if self.base_view_vector is not None:
            self.base_data_timestamp = self.get_timestamp()

        self.update_timer()

    def stop_tracking(self):
        if self.tracking_timer is None:
            return

        self.window.after_cancel(self.tracking_timer)
        self.tracking_timer = None

        if self.base_view_vector is None:
            return

        final_vec, roll = recalc_local_view_vector(self.base_view_vector, 0.0, self.observer_ll[0], self.get_delta())
        self.set_view_vector(final_vec)
        self.base_data_timestamp = None

    def update_tracking(self):
        # TODO: possibly add an object tracking mode where we redo fetch_stellarium() every time, calculate
        # velocity based on the delta, etc.  Roll is going to be tricky in this mode.

        self.update_ui_info()
        self.update_gimbal(continuous=True)
        self.update_timer()

    def exit_application(self):
        self.stop_tracking()
        self.window.destroy()

# Main application loop
def main():
    window = tk.Tk()
    app = GimbalGoToApp(window)
    window.protocol("WM_DELETE_WINDOW", app.exit_application)
    window.mainloop()

if __name__ == "__main__":
    main()
