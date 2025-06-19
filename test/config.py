"""
camera_ground_sn
camera_robot_sn
emergency
speed_ratio
tcp
"""

import numpy as np
"""
state
    -1: init
     0: stopping
     1: stopped
     2: standby
     3: working
"""
"""
camera
"""
#camera_ground_sn = "140122073620" # ground camera SN
#camera_robot_sn = "130322272986" # on robot camera SN
camera_ground_sn = None # ground camera SN
camera_robot_sn = "130322272239" # on robot camera SN
camera_ground_preset_path = "Intel_RealSense_D435.json" # preset path
camera_robot_preset_path = "Intel_RealSense_D405.json" # preset path
camera_ground_exposure = None
#camera_robot_exposure = 90000
camera_robot_exposure = None

"""
detection
"""
detection_prm ={
    "cardamom": {
        'roi': {'corners': [[228.85, 307.03], [445.36, 4.49], [753.65, 216.7], [577.28, 476.23], [472.61, 471.93]], 'inv': False, 'crop': True, 'offset': 0}, 
        'limit': {'xyz': {'x': [-1000, 1000], 'y': [-1000, 1000], 'z': [0, 87], 'inv': 0}}
        },
    "digit":{
        'roi': {'corners': [[386.58, 255.42], [386.58, 315.64], [528.54, 319.95], [535.71, 256.86]], 'inv': False, 'crop': False, 'offset': 0}, 
        'detection': {'cmd': 'ocr', 'conf': 0.03}
    }
}

"""
weight
"""
cycles = 100000

"""
robot
"""
robot_ip = "192.168.1.100"# robot ip address
#tcp_length = 97.5+0 # Make sure to put the right toolhead length
tcp_length = 97.5+1 # Make sure to put the right toolhead length
output = [0, [0, 1]] # [output_pin, [off_action, on_action]]
#emergency=["in0",[0, 1]] # [pin_index, [off_state, on_state]]
emergency = None
safe_z = 275

"""
speed
"""
speed_ratio = 0.5
vel = 150*speed_ratio #100
accel = 2000*speed_ratio #800 
jerk = 4000*speed_ratio #1000 

"""
kinematic
"""
robot_model = "dorna_ta"

T_cam_2_j4 = np.matrix([[ 5.25873615e-03, -9.99894519e-01,  1.34620306e-02,
          4.65174596e+01],
        [ 9.99959617e-01,  5.35678348e-03, -7.35796480e-03,
          3.20776662e+01],
        [ 7.28773209e-03, -1.35001806e-02,  9.99882310e-01,
         -4.24772615e+00],
        [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
          1.00000000e+00]])
"""
T_cam_2_j4 = np.matrix([[0.010943887103459813, -0.9999341003666327, -0.0038674619037596144, 50.27236517370847], [0.9997735117091237, 0.010878692403868672, -0.018211214507866908, 33.50026421799676], [0.018252578444525054, 0.003667284493332018, 0.999826682182753, -4.748022881116398], [0.0, 0.0, 0.0, 1.0]])
"""
"""
detection
"""
trained_model = "yolov8n-seg_cardamom_good_bad_nano_tile_null_removed_v3.pt"
detection_conf = 0.4
max_det = 10
area_thr = [500, 12000] # min and max area
#xyz_thr_old = [[190,290], [-260, -160], [3, 90]] # with respect to the robot base
xyz_thr = [[90,175], [-250, -170], [2, 80]] # with respect to the robot base
quality_roi = [[560, 190], [560, 1], [847, 1], [847, 190]]


bin_roi = [[692, 185], [517, 421], [768, 620], [951, 345]]
#ocr_roi = [[582, 602], [587, 545], [798, 539], [793, 597]]
ocr_roi = [[570, 388], [773, 398], [759, 455], [563, 449]]
#quality_2_roi = [[271, 241], [613, 172], [684, 520], [331, 584]]
quality_2_roi = [[533, 11], [876, 12], [865, 349], [534, 350]]
"""
bin_roi = [[458, 123], [342, 280], [508, 413], [630, 230]]
ocr_roi = [[377, 258], [512, 265], [502, 303], [372, 299]]
quality_2_roi = [[353, 7], [580, 8], [573, 232], [353, 233]]
"""



bin_image_thr = 5
mix_thr = 5
quality_more = {"multi_bottom":False, "second_top":False} # number of time we will take quality 

"""
weight
"""
weight_thr = 0.35 # gram
font_path = "simfang.ttf"
tare_cycle = 100

"""
motion
"""
main_loop = [
    {"cmd": "output", "out"+str(output[0]): output[1][1], "queue":0},
    {"cmd": "sleep", "time": 0.5},
    {"cmd": "output", "out"+str(output[0]): output[1][0], "queue":0},
    {"cmd": "sleep", "time": 0.5},
    {"cmd": "output", "out"+str(output[0]): output[1][1], "queue":0},
    {"cmd": "sleep", "time": 0.5},
    {"cmd": "output", "out"+str(output[0]): output[1][0], "queue":0},
    {"cmd": "sleep", "time": 0.5},
    {"cmd": "output", "out"+str(output[0]): output[1][1], "queue":0},
    {"cmd": "sleep", "time": 0.5},
    {"cmd": "output", "out"+str(output[0]): output[1][0], "queue":0},
    {"cmd": "sleep", "time": 0.5},
]

work_init = [
    {"cmd": "output", "out"+str(output[0]): output[1][0], "queue":0},
    {"cmd": "motor", "motor": 1},
    {"cmd": "sleep", "time": 0.5},
    {"cmd":"jmove","rel":0,"j2":-150,"j5":0, "vel":100, "accel": 800, "jerk": 1000, "cont":0},
    {"cmd":"jmove","rel":0, "j4":-10},
    {"cmd":"jmove","rel":0, "j0":0, "j3":0},
]

alarm_init = [
    work_init[0],
    work_init[1],
    work_init[2],
    {"cmd":"lmove","rel":0, "z":safe_z, "vel":100, "accel": 2000, "jerk": 4000, "cont":0},
    {"cmd":"jmove","rel":0,"j0":0,"j1":90,"j2":-150,"j3":0,"j4":-10,"j5":0, "vel":100, "accel": 800, "jerk": 1000, "cont":0}
]


start = [
    {"cmd": "output", "out"+str(output[0]): output[1][0], "queue":0},
    {"cmd":"motor", "motor": 1},
    {"cmd":"sleep", "time": 0.5},
]

bin_image = [
    {"cmd":"jmove","rel":0, "j0":-54.470215,"j1":59.47998,"j2":-141.394043,"j3":0,"j4":-8.10791,"j5":0, "vel":vel, "accel":accel,"jerk":jerk, "cont": 0},
    {"cmd": "output", "out"+str(output[0]): output[1][0], "queue":0},
    {"cmd":"sleep", "time": 0.2},
]

# after bin imaging
pick = [
    {"cmd":"lmove","rel":0,"vel": 500, "accel": 4000,"jerk":8000},
    {"cmd": "output", "out"+str(output[0]): output[1][1], "queue":0},
    {"cmd":"lmove","rel":1, "z": -11},
    {"cmd":"cmove","rel":1, "x": 2, "y":2, "z":-2, "mx":2, "my":-2, "mz":-2, "vel":40},
]

# after bin imaging
"""
bin_mix = [
    bin_image[0],
    {"cmd":"jmove","rel":0,"j0":-55.50293,"j1":23.554688,"j2":-111.225586,"j3":-28.894043,"j4":1.032715,"j5":-0.74707},
    {"cmd":"jmove","rel":0,"j0":-61.743164,"j1":25.026855,"j2":-99.645996,"j3":-17.336426,"j4":-12.854004,"j5":-6.328125},
    {"cmd":"jmove","rel":0,"j0":-71.30127,"j1":25.510254,"j2":-102.502441,"j3":-38.649902,"j4":-8.942871,"j5":-9.250488},
    {"cmd":"jmove","rel":0,"j0":-67.961426,"j1":22.543945,"j2":-114.389648,"j3":-41.550293,"j4":-0.439453,"j5":-6.350098},
    {"cmd":"jmove","rel":0,"j0":-61.743164,"j1":25.026855,"j2":-99.645996,"j3":-17.336426,"j4":-12.854004,"j5":-6.328125},
    {"cmd":"jmove","rel":0,"j0":-71.30127,"j1":25.510254,"j2":-102.502441,"j3":-38.649902,"j4":-8.942871,"j5":-9.250488},
    {"cmd":"jmove","rel":0,"j0":-55.50293,"j1":23.554688,"j2":-111.225586,"j3":-28.894043,"j4":1.032715,"j5":-0.74707},
    bin_image[0],
]
"""

bin_mix = [
    bin_image[0],
    {"cmd":"jmove","rel":0,"j0":-55.50293,"j1":25.554688,"j2":-111.225586,"j3":-28.894043,"j4":1.032715,"j5":-0.74707},
    {"cmd":"jmove","rel":0,"j0":-61.743164,"j1":27.026855,"j2":-99.645996,"j3":-17.336426,"j4":-12.854004,"j5":-6.328125},
    {"cmd":"jmove","rel":0,"j0":-71.30127,"j1":27.510254,"j2":-102.502441,"j3":-38.649902,"j4":-8.942871,"j5":-9.250488},
    {"cmd":"jmove","rel":0,"j0":-67.961426,"j1":24.543945,"j2":-114.389648,"j3":-41.550293,"j4":-0.439453,"j5":-6.350098},
    {"cmd":"jmove","rel":0,"j0":-61.743164,"j1":27.026855,"j2":-99.645996,"j3":-17.336426,"j4":-12.854004,"j5":-6.328125},
    {"cmd":"jmove","rel":0,"j0":-71.30127,"j1":27.510254,"j2":-102.502441,"j3":-38.649902,"j4":-8.942871,"j5":-9.250488},
    {"cmd":"jmove","rel":0,"j0":-55.50293,"j1":25.554688,"j2":-111.225586,"j3":-28.894043,"j4":1.032715,"j5":-0.74707},
    bin_image[0],
]

# after pick
bad_bin_after_pick = [
    {**bin_image[0], **{"vel": 100, "accel": 800,"jerk": 1000, "cont":1, "corner":20}},
    {"cmd":"jmove","rel":0,"j0":-39.089355,"j1":28.366699,"j2":-60.820312,"j3":3.515625,"j4":-57.98584,"j5":-0.021973, "cont":0},
    {"cmd": "output", "out"+str(output[0]): output[1][0], "queue":0},
    {"cmd":"sleep", "time": 0.1},
]

# affter pick
quality = [
    {"cmd":"lmove","rel":0, "z": 180, "vel": 500, "accel": 4000,"jerk":8000},
    #{**bin_image[0], **{"vel": vel, "accel": accel,"jerk": jerk, "cont":0}},
    {"cmd":"jmove","rel":0,"j0":-55.50293,"j1":61.962891,"j2":-141.987305,"j3":0,"j4":-42.011719,"j5":-90,"vel": vel, "accel": accel,"jerk": jerk,"cont":0},
    {"cmd":"sleep", "time": 0.2},
]

# after pick
bad_bin_after_quality = [
    {**bad_bin_after_pick[1], **{"vel": vel, "accel": accel,"jerk": jerk, "cont":0}},
    {"cmd": "output", "out"+str(output[0]): output[1][0], "queue":0},
    {"cmd":"sleep", "time": 0.1},
]

# after quality_1
quality_2=[
    {"cmd":"jmove","rel":0,"j0":-9.667969,"j1":73.740234,"j2":-118.146973,"j3":1.318359,"j4":-44.780273,"j5":0, "vel": vel, "accel": accel, "jerk": jerk, "cont": 0},
    {"cmd":"jmove","rel":0,"j0":-8.129883,"j1":60.88623,"j2":-101.66748,"j3":1.208496,"j4":-48.383789,"j5":0},
    {"cmd":"jmove","rel":0,"j0":-8.129883,"j1":59.985352,"j2":-102.23877,"j3":1.252441,"j4":-46.911621,"j5":0},
    {"cmd": "output", "out"+str(output[0]): output[1][0], "queue":0},
    {"cmd": "sleep", "time":0.1},
    {"cmd":"jmove","rel":0,"j0":-8.129883,"j1":60.88623,"j2":-101.66748,"j3":1.208496,"j4":-48.383789,"j5":0},
    #{"cmd":"jmove","rel":0,"j0":-9.667969,"j1":73.740234,"j2":-118.146973,"j3":1.318359,"j4":-44.780273,"j5":0},
    {"cmd":"jmove","rel":0,"j0":0,"j1":80,"j2":-125,"j3":0,"j4":-50,"j5":0},
    {"cmd":"sleep", "time": 0.2},
]


# after quality_2
weight = [
    {"cmd":"jmove","rel":0,"j0":0,"j1":92,"j2":-147,"j3":-2,"j4":-40,"j5":0},
    {"cmd":"sleep", "time": 0.2},
]

# after weight
"""
drop_good = [
    {"cmd":"jmove","rel":0,"j0":-17.028809,"j1":56.887207,"j2":-87.297363,"j3":-17.358398,"j4":-58.535156,"j5":-18.259277, "vel": vel, "accel": accel,"jerk": jerk, "cont":0},
    {"cmd":"jmove","rel":0,"j0":-13.798828,"j1":49.724121,"j2":-87.714844,"j3":-27.927246,"j4":-53.679199,"j5":-7.668457,"vel":50, "accel": 800, "jerk":1000, "cont":1, "corner":15},
    {"cmd":"jmove","rel":0,"j0":-9.118652,"j1":50.603027,"j2":-88.417969,"j3":-33.00293,"j4":-51.877441,"j5":0.065918},
    {"cmd":"jmove","rel":0,"j0":-12.480469,"j1":57.568359,"j2":-91.296387,"j3":-9.558105,"j4":-56.711426,"j5":-16.479492},{"cmd":"jmove","rel":0,"j0":-16.699219,"j1":45.812988,"j2":-75.783691,"j3":-26.696777,"j4":-69.455566,"j5":-12.524414, "cont":0},
    {"cmd":"jmove","rel":0,"j0":-17.028809,"j1":56.887207,"j2":-87.297363,"j3":-17.358398,"j4":-58.535156,"j5":-18.259277, "cont":0},
]
"""
drop_good = [
    {"cmd":"jmove","rel":0,"j0":-17.028809,"j1":56.887207,"j2":-87.297363,"j3":-17.358398,"j4":-58.535156,"j5":-18.259277,"vel": vel, "accel": accel,"jerk": jerk, "cont":0},
    {"cmd":"jmove","rel":0,"j0":-13.798828,"j1":49.724121,"j2":-87.714844,"j3":-27.927246,"j4":-53.679199,"j5":-7.668457,"vel":50, "accel": 800, "jerk":1000, "cont":1, "corner":15},
    {"cmd":"jmove","rel":0,"j0":-9.118652,"j1":50.603027,"j2":-88.417969,"j3":-33.00293,"j4":-51.877441,"j5":0.065918},
    {"cmd":"jmove","rel":0,"j0":-12.480469,"j1":57.568359,"j2":-91.296387,"j3":-9.558105,"j4":-56.711426,"j5":-16.479492},{"cmd":"jmove","rel":0,"j0":-16.699219,"j1":45.812988,"j2":-75.783691,"j3":-26.696777,"j4":-69.455566,"j5":-12.524414, "vel": 125},
    {**bin_image[0], **{"vel": 125, "accel": 800,"jerk": 1000, "cont":0}},
]

# after weight
"""
drop_bad = [
    {"cmd":"jmove","rel":0,"j0":5.273438,"j1":58.183594,"j2":-100.437012,"j3":5.932617,"j4":-48.713379,"j5":-7.822266, "vel": vel, "accel": accel,"jerk": jerk, "cont":0},
    {"cmd":"jmove","rel":0,"j0":2.219238,"j1":47.06543,"j2":-107.358398,"j3":19.6875,"j4":-38.891602,"j5":-15.776367, "vel":50, "accel": 800, "jerk":1000, "cont":1, "corner":15},
    {"cmd":"jmove","rel":0,"j0":-0.307617,"j1":44.143066,"j2":-102.98584,"j3":21.401367,"j4":-49.350586,"j5":-9.42627},
    {"cmd":"jmove","rel":0,"j0":-1.582031,"j1":58.117676,"j2":-102.019043,"j3":7.580566,"j4":-47.197266,"j5":-11.491699},
    {"cmd":"jmove","rel":0,"j0":6.37207,"j1":47.724609,"j2":-104.875488,"j3":9.294434,"j4":-37.727051,"j5":-11.601562},
    {"cmd":"jmove","rel":0,"j0":5.273438,"j1":58.183594,"j2":-100.437012,"j3":5.932617,"j4":-48.713379,"j5":-7.822266, "cont":0},
    {"cmd":"jmove","rel":0,"j0":-44.472656,"j1":82.595215,"j2":-136.07666,"j3":0,"j4":-36.5625,"j5":0 , "vel": vel, "accel": accel,"jerk": jerk, "cont":0}
]
"""
drop_bad = [
    {"cmd":"jmove","rel":0,"j0":5.273438,"j1":58.183594,"j2":-100.437012,"j3":5.932617,"j4":-48.713379,"j5":-7.822266, "vel": vel, "accel": accel,"jerk": jerk, "cont":0},
    {"cmd":"jmove","rel":0,"j0":2.219238,"j1":47.06543,"j2":-107.358398,"j3":19.6875,"j4":-38.891602,"j5":-15.776367, "vel":50, "accel": 800, "jerk":1000, "cont":1, "corner":15},
    {"cmd":"jmove","rel":0,"j0":-0.307617,"j1":44.143066,"j2":-102.98584,"j3":21.401367,"j4":-49.350586,"j5":-9.42627},
    {"cmd":"jmove","rel":0,"j0":-1.582031,"j1":58.117676,"j2":-102.019043,"j3":7.580566,"j4":-47.197266,"j5":-11.491699},
    {"cmd":"jmove","rel":0,"j0":6.37207,"j1":47.724609,"j2":-104.875488,"j3":9.294434,"j4":-37.727051,"j5":-11.601562},
    #{"cmd":"jmove","rel":0,"j0":5.910645,"j1":77.651367,"j2":-124.299316,"j3":5.097656,"j4":-45.922852,"j5":-10.524902, "vel":100, "accel":800, "jerk":1000},
    {"cmd":"jmove","rel":0,"j0":5.910645,"j1":77.651367,"j2":-124.299316,"j3":5.097656,"j4":-45.922852,"j5":-10.524902, "vel":75, "accel":600, "jerk":1000},
    {**bin_image[0], **{"vel": 75, "accel": 800,"jerk": 1000, "cont":0}},
]


tare = [
    {**bin_image[0], **{"vel": 100, "accel": 800,"jerk": 1000, "cont":0}},
    {"cmd":"jmove","rel":0,"j0":-29.575195,"j1":15.358887,"j2":-126.320801,"j3":-32.563477,"j4":-14.282227,"j5":99.18457},
    {"cmd":"jmove","rel":0,"j0":-29.729004,"j1":13.666992,"j2":-123.969727,"j3":-33.793945,"j4":-15.314941,"j5":98.986816},
    {"cmd":"jmove","rel":0,"j0":-29.575195,"j1":15.358887,"j2":-126.320801,"j3":-32.563477,"j4":-14.282227,"j5":99.18457},
    bin_image[0],
]

