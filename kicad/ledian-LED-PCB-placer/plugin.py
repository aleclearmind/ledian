import pcbnew
import wx
import sys
import os

# board parameters
# all distance measures are in milli meters

ROWS = 12
COLS = 40

BOARD_OFF = 25

BOARD_MARGIN = 0

CAP_OFF_X =  -4
CAP_OFF_Y =  -4.5

VIA_SIGNAL_DRILL = 0.4
VIA_SIGNAL_WIDTH = 0.8
VIA1_OFF_X = -4
VIA1_OFF_Y = -2.45
VIA2_OFF_X = -4
VIA2_OFF_Y =  2.45

EDGE_CUT_LINES_WIDTH = 0.25

LEDS_PER_METER = 80
LEDS_FORM_FACTOR = 2.17

VCC_SIGNAL_TRACK_WIDTH = 0.75
DATA_SIGNAL_TRACK_WIDTH = 0.75

POWERPAD_OFF_X = 10
POWERPAD_OFF_Y = 9

POWER_PLANE_DISTANCE_BOARD = 1

VIAS_PER_CELL_TO_BOTTOM = 7

VIA_POWERPLANE_DRILL = 0.4
VIA_POWERPLANE_WIDTH = 0.8

HEIGHT_SCREW_CUTOFF = 4
HEIGHT_GRID_CUTOFF = 4

# derived variables from parameters

BOARD_START = BOARD_OFF + BOARD_MARGIN

LEDS = ROWS * COLS
LEDS_DISTANCE_X = 1000 / (LEDS_PER_METER)
LEDS_DISTANCE_Y = (LEDS_DISTANCE_X * LEDS_FORM_FACTOR)

leds = [{} for _ in range(LEDS)]
pads = [{} for _ in range(ROWS*2)]

def CreateTrack(p1, p2, net, width, layer):
    pcb = pcbnew.GetBoard()

    track = pcbnew.PCB_TRACK(pcb)

    track.SetWidth(pcbnew.FromMM(width))
    track.SetLayer(layer)
    track.SetNet(net)

    track.SetStart(p1)
    track.SetEnd(p2)

    pcb.Add(track)

def CreateTopTrack(p1, p2, net, width):
    CreateTrack(p1, p2, net, width, pcbnew.F_Cu)

def CreateBottomTrack(p1, p2, net, width):
    CreateTrack(p1, p2, net, width, pcbnew.B_Cu)

def CreateLineSilkScreen(x1, y1, x2, y2):
    pcb = pcbnew.GetBoard()

    line = pcbnew.PCB_SHAPE(pcb)
    line.SetShape(pcbnew.S_SEGMENT)
    line.SetLayer(pcbnew.F_SilkS)
    line.SetWidth(pcbnew.FromMM(0.25))

    line.SetStart(pcbnew.VECTOR2I_MM(x1, y1))
    line.SetEnd(pcbnew.VECTOR2I_MM(x2, y2))

    pcb.Add(line)

def CreateBoardOutline(x, y, width, height):
    pcb = pcbnew.GetBoard()

    points = (
        pcbnew.VECTOR2I_MM(x,         y         ),
        pcbnew.VECTOR2I_MM(x + width, y         ),
        pcbnew.VECTOR2I_MM(x + width, y + height),
        pcbnew.VECTOR2I_MM(x,         y + height),
    )

    edge = pcbnew.PCB_SHAPE(pcb)
    edge.SetShape(pcbnew.SHAPE_T_POLY)
    edge.SetFilled(False)
    edge.SetLayer(pcbnew.Edge_Cuts)
    edge.SetWidth(pcbnew.FromMM(0.25))

    v = []
    for point in points:
        v.append(point)
    edge.SetPolyPoints(v)

    pcb.Add(edge)

def CreatePowerPlane(x, y, width, height, layer, net):
    pcb = pcbnew.GetBoard()

    points = (
        pcbnew.VECTOR2I_MM(x,         y         ),
        pcbnew.VECTOR2I_MM(x + width, y         ),
        pcbnew.VECTOR2I_MM(x + width, y + height),
        pcbnew.VECTOR2I_MM(x,         y + height),
    )

    chain = pcbnew.SHAPE_LINE_CHAIN()
    for (x, y) in points:
        chain.Append(x, y)
    chain.SetClosed(True)

    zone = pcbnew.ZONE(pcb)
    zone.SetNet(net)
    zone.AddPolygon(chain)
    zone.SetLayer(layer)
    zone.SetIsFilled(True)
    zone.SetThermalReliefGap(pcbnew.FromMM(0.65))
    zone.SetThermalReliefSpokeWidth(pcbnew.FromMM(0.75))
#    zone.SetCornerRadius(pcbnew.FromMM(1))
#    zone.SetCornerSmoothingType(2)
    pcb.Add(zone)

def CreateExclusionZone(x, y, width, height):
    pcb = pcbnew.GetBoard()

    points = (
        pcbnew.VECTOR2I_MM(x,         y         ),
        pcbnew.VECTOR2I_MM(x + width, y         ),
        pcbnew.VECTOR2I_MM(x + width, y + height),
        pcbnew.VECTOR2I_MM(x,         y + height),
    )

    chain = pcbnew.SHAPE_LINE_CHAIN()
    for (x, y) in points:
        chain.Append(x, y)
    chain.SetClosed(True)

    lset = pcbnew.LSET()
    lset.AddLayer(pcbnew.F_Cu)
    lset.AddLayer(pcbnew.B_Cu)

    zone = pcbnew.ZONE(pcb)
    zone.AddPolygon(chain)
    zone.SetIsRuleArea(True)
    zone.SetDoNotAllowCopperPour(True)
    zone.SetDoNotAllowTracks(False)
    zone.SetLayerSet(lset)
    zone.SetAssignedPriority(100)
#    zone.SetCornerRadius(pcbnew.FromMM(1))
#    zone.SetCornerSmoothingType(2)
    pcb.Add(zone)

def GetNetInfoByName(name):
    pcb = pcbnew.GetBoard()
    nets = pcb.GetNetsByName()
    net = nets.find(name)
    netinfo = net.value()[1]

    return netinfo

def CreateGndPowerPlane(x, y, width, height, layer):
    gndnet = GetNetInfoByName("GND")
    CreatePowerPlane(x, y, width, height, layer, gndnet)

def CreateVddPowerPlane(x, y, width, height, layer):
    vddnet = GetNetInfoByName("VDD")
    CreatePowerPlane(x, y, width, height, layer, vddnet)

def CreateVddTopPowerPlane(x, y, width, height):
    CreateVddPowerPlane(x, y, width, height, pcbnew.F_Cu)

def CreateVddBottomPowerPlane(x, y, width, height):
    CreateVddPowerPlane(x, y, width, height, pcbnew.B_Cu)

def CreateGndTopPowerPlane(x, y, width, height):
    CreateGndPowerPlane(x, y, width, height, pcbnew.F_Cu)

def CreateGndBottomPowerPlane(x, y, width, height):
    CreateGndPowerPlane(x, y, width, height, pcbnew.B_Cu)

def AddSignalVia(pos, drill, width, net):
    pcb = pcbnew.GetBoard()
    via = pcbnew.PCB_VIA(pcb)
    via.SetPosition(pos)
    via.SetDrill(pcbnew.FromMM(drill))
    via.SetWidth(pcbnew.FromMM(width))
    via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    via.SetViaType(pcbnew.VIATYPE_THROUGH)
    via.SetNet(net)
    pcb.Add(via)

def AddGndVia(pos, drill, width):
    gnd_net = GetNetInfoByName("GND")
    AddSignalVia(pos, drill, width, gnd_net)

def AddVddVia(pos, drill, width):
    vdd_net = GetNetInfoByName("VDD")
    AddSignalVia(pos, drill, width, vdd_net)

def PopulateComponents():
    pcb = pcbnew.GetBoard()
    footprints = pcb.GetFootprints()

    for f in footprints:
        ref = f.GetReference()
        pos = f.GetPosition()

        if ref.startswith("D"):
            led_num = int(ref[1:]) - 1
            leds[led_num]["led"] = f

        if ref.startswith("C"):
            cap_num = int(ref[1:]) - 1
            leds[cap_num]["cap"] = f

        if ref.startswith("JV_"):
            pad_num = int(ref[3:]) - 1
            pads[pad_num]["VDD"] = f

        if ref.startswith("JN_"):
            pad_num = int(ref[3:]) - 1
            pads[pad_num]["GND"] = f

din1_offsets = [
    pcbnew.VECTOR2I_MM(0, 1.25),
    pcbnew.VECTOR2I_MM(-1.9, 1.9),
    pcbnew.VECTOR2I_MM(-7, 0),
]

din2_offsets = [
    pcbnew.VECTOR2I_MM(0, 1.25),
    pcbnew.VECTOR2I_MM(0.5, 0.5),
    pcbnew.VECTOR2I_MM(5, 0),
]

dout_offsets = [
    pcbnew.VECTOR2I_MM(0, -1.25),
    pcbnew.VECTOR2I_MM(0.5, -0.5),
    pcbnew.VECTOR2I_MM(5, 0),
]

bin_offsets = [
    pcbnew.VECTOR2I_MM(0, 1.25),
    pcbnew.VECTOR2I_MM(-0.5, 0.5),
    pcbnew.VECTOR2I_MM(-5, 0),
]

def CreateTrace(start_pos, offsets, net, width):
    poses = []
    poses.append(start_pos)

    last_pos = start_pos

    for offset in offsets:
        cur_pos = last_pos + offset
        poses.append(cur_pos)
        last_pos = cur_pos

    for start, end in zip(poses, poses[1:]):
        CreateTopTrack(start, end, net, width)

def PlaceLEDsAndCaps():
    for itm in leds:
        led = itm["led"]
        led_ref = led.GetReference()
        led_pos = led.GetPosition()

        cap = itm["cap"]
        cap_ref = cap.GetReference()
        cap_pos = cap.GetPosition()

        led_num = int(led_ref[1:]) - 1
        led_row = int(led_num / COLS)
        led_col = led_num % COLS

        # rewrite this huge pile of crap
        if led_num % COLS == 0:
            if led_row % 2 == 0:
                border_left = True
                border_right = False
            else:
                border_left = False
                border_right = True
        elif led_num % COLS == (COLS - 1):
            if led_row % 2 == 0:
                border_left = False
                border_right = True
            else:
                border_left = True
                border_right = False
        else:
            border_left = False
            border_right = False

        print("# placing {0}, {1}".format(led_ref, cap_ref))
        print("border_left = {0}".format(border_left))
        print("border_right = {0}".format(border_right))
        print("led_row = {0}".format(led_row))
        print("led_num = {0}".format(led_num))

        if led_row % 2 == 0:
            led_x = BOARD_START + led_col*LEDS_DISTANCE_X + LEDS_DISTANCE_X/2
            led_y = BOARD_START + led_row*LEDS_DISTANCE_Y + LEDS_DISTANCE_Y/2
            led_rot = 90
        else:
            led_x = BOARD_START + (COLS-led_col) * LEDS_DISTANCE_X - LEDS_DISTANCE_X/2
            led_y = BOARD_START + led_row*LEDS_DISTANCE_Y + LEDS_DISTANCE_Y/2
            led_rot = -90

        led.SetPosition(pcbnew.VECTOR2I_MM(led_x,led_y))
        led.SetOrientationDegrees(led_rot)

        if led_row % 2 == 0:
            cap_x = led_x + CAP_OFF_X
            cap_y = led_y + CAP_OFF_Y
            cap_rot = 90
        else:
            cap_x = led_x - CAP_OFF_X
            cap_y = led_y - CAP_OFF_Y
            cap_rot = -90

        cap.SetPosition(pcbnew.VECTOR2I_MM(cap_x,cap_y))
        cap.SetOrientationDegrees(cap_rot)

        cap_pad_gnd = GetPadByName(cap, "2")
        cap_pad_gnd.SetThermalGap(pcbnew.FromMM(0.5))
        cap_pad_gnd.SetThermalSpokeWidth(pcbnew.FromMM(0.5))

        led_vcc_pad = GetPadByName(led, "1")
        led_vcc_net = led_vcc_pad.GetNet()
        led_vcc_pos = led_vcc_pad.GetPosition()

        led_din_pad = GetPadByName(led, "4")
        led_din_net = led_din_pad.GetNet()
        led_din_pos = led_din_pad.GetPosition()

        led_dout_pad = GetPadByName(led, "3")
        led_dout_net = led_dout_pad.GetNet()
        led_dout_pos = led_dout_pad.GetPosition()

        led_bin_pad = GetPadByName(led, "6")
        led_bin_net = led_bin_pad.GetNet()
        led_bin_pos = led_bin_pad.GetPosition()

        if led_row % 2 == 0:
            via1_x = led_x + VIA1_OFF_X
            via1_y = led_y + VIA1_OFF_Y
            via2_x = led_x + VIA2_OFF_X
            via2_y = led_y + VIA2_OFF_Y
        else:
            via1_x = led_x - VIA2_OFF_X
            via1_y = led_y + VIA2_OFF_Y
            via2_x = led_x - VIA1_OFF_X
            via2_y = led_y + VIA1_OFF_Y

        via1_pos = pcbnew.VECTOR2I_MM(via1_x, via1_y)
        via2_pos = pcbnew.VECTOR2I_MM(via2_x, via2_y)

        vcc_cap_pad = GetPadByNet(cap, led_vcc_net)
        vcc_cap_pos = vcc_cap_pad.GetPosition()

        AddSignalVia(via1_pos, VIA_SIGNAL_DRILL, VIA_SIGNAL_WIDTH, led_vcc_net)
        AddSignalVia(via2_pos, VIA_SIGNAL_DRILL, VIA_SIGNAL_WIDTH, led_vcc_net)

        CreateBottomTrack(via1_pos, via2_pos, led_vcc_net, VCC_SIGNAL_TRACK_WIDTH)
        CreateTopTrack(via2_pos, led_vcc_pos, led_vcc_net, VCC_SIGNAL_TRACK_WIDTH)
        CreateTopTrack(via1_pos, vcc_cap_pos, led_vcc_net, VCC_SIGNAL_TRACK_WIDTH)

        invert = lambda x: -x

        if led_row % 2 == 0:
            if not border_left:
                CreateTrace(led_din_pos, din1_offsets, led_din_net, DATA_SIGNAL_TRACK_WIDTH)
                CreateTrace(led_bin_pos, bin_offsets, led_bin_net, DATA_SIGNAL_TRACK_WIDTH)
            if not border_right:
                CreateTrace(led_din_pos, din2_offsets, led_din_net, DATA_SIGNAL_TRACK_WIDTH)
                CreateTrace(led_dout_pos, dout_offsets, led_dout_net, DATA_SIGNAL_TRACK_WIDTH)
            if border_right:
                track1_pos = led_din_pos + pcbnew.VECTOR2I_MM(0, 1.25)
                via_pos = led_din_pos + pcbnew.VECTOR2I_MM(2, 3.25)
                track2_pos = via_pos + pcbnew.VECTOR2I_MM(0, 3)
                track3_pos = track2_pos + pcbnew.VECTOR2I_MM(-2, 2)
                track4_pos = track3_pos + pcbnew.VECTOR2I_MM(0, LEDS_DISTANCE_Y/2 - 6)

                AddSignalVia(via_pos, VIA_SIGNAL_DRILL, VIA_SIGNAL_WIDTH, led_din_net)
                CreateTopTrack(track1_pos, via_pos, led_din_net, DATA_SIGNAL_TRACK_WIDTH)
                CreateBottomTrack(via_pos, track2_pos, led_din_net, DATA_SIGNAL_TRACK_WIDTH)
                CreateBottomTrack(track2_pos, track3_pos, led_din_net, DATA_SIGNAL_TRACK_WIDTH)
                CreateBottomTrack(track3_pos, track4_pos, led_din_net, DATA_SIGNAL_TRACK_WIDTH)

                via_pos = led_dout_pos + pcbnew.VECTOR2I_MM(0, -1.7)
                track1_pos = via_pos + pcbnew.VECTOR2I_MM(0, 2.5)
                track2_pos = track1_pos + pcbnew.VECTOR2I_MM(-1.6, 1.6)
                track3_pos = track2_pos + pcbnew.VECTOR2I_MM(0, LEDS_DISTANCE_Y/2 - 5)

                CreateTopTrack(led_dout_pos, via_pos, led_dout_net, DATA_SIGNAL_TRACK_WIDTH)
                AddSignalVia(via_pos, VIA_SIGNAL_DRILL, VIA_SIGNAL_WIDTH, led_dout_net)
                CreateBottomTrack(via_pos, track1_pos, led_dout_net, DATA_SIGNAL_TRACK_WIDTH)
                CreateBottomTrack(track1_pos, track2_pos, led_dout_net, DATA_SIGNAL_TRACK_WIDTH)
                CreateBottomTrack(track2_pos, track3_pos, led_dout_net, DATA_SIGNAL_TRACK_WIDTH)
            if border_left and led_row:
                via_pos = led_bin_pos + pcbnew.VECTOR2I_MM(0, 1.75)
                track1_pos = via_pos + pcbnew.VECTOR2I_MM(0, -LEDS_DISTANCE_Y/2)

                CreateTopTrack(led_bin_pos, via_pos, led_bin_net, DATA_SIGNAL_TRACK_WIDTH)
                AddSignalVia(via_pos, VIA_SIGNAL_DRILL, VIA_SIGNAL_WIDTH, led_bin_net)
                CreateBottomTrack(via_pos, track1_pos, led_bin_net, DATA_SIGNAL_TRACK_WIDTH)

                track1_pos = led_din_pos + pcbnew.VECTOR2I_MM(0, 1.75)
                via_pos = track1_pos + pcbnew.VECTOR2I_MM(-1.6, 0)
                track2_pos = via_pos + pcbnew.VECTOR2I_MM(0, -LEDS_DISTANCE_Y/2)

                CreateTopTrack(track1_pos, via_pos, led_din_net, DATA_SIGNAL_TRACK_WIDTH)
                AddSignalVia(via_pos, VIA_SIGNAL_DRILL, VIA_SIGNAL_WIDTH, led_din_net)
                CreateBottomTrack(via_pos, track2_pos, led_din_net, DATA_SIGNAL_TRACK_WIDTH)
        else:
            if not border_left:
                CreateTrace(led_din_pos, map(invert, din2_offsets), led_din_net, DATA_SIGNAL_TRACK_WIDTH)
                CreateTrace(led_dout_pos, map(invert, dout_offsets), led_dout_net, DATA_SIGNAL_TRACK_WIDTH)
            if not border_right:
                CreateTrace(led_din_pos, map(invert, din1_offsets), led_din_net, DATA_SIGNAL_TRACK_WIDTH)
                CreateTrace(led_bin_pos, map(invert, bin_offsets), led_bin_net, DATA_SIGNAL_TRACK_WIDTH)
            if border_right:
                via_pos = led_bin_pos + pcbnew.VECTOR2I_MM(0, -2.5)
                track1_pos = via_pos + pcbnew.VECTOR2I_MM(0, - LEDS_DISTANCE_Y/2)
                CreateTopTrack(led_bin_pos, via_pos, led_bin_net, DATA_SIGNAL_TRACK_WIDTH)
                AddSignalVia(via_pos, VIA_SIGNAL_DRILL, VIA_SIGNAL_WIDTH, led_bin_net)
                CreateBottomTrack(via_pos, track1_pos, led_bin_net, DATA_SIGNAL_TRACK_WIDTH)

                via_pos = led_din_pos + pcbnew.VECTOR2I_MM(1.6, -2.5)
                track1_pos = via_pos + pcbnew.VECTOR2I_MM(0, - LEDS_DISTANCE_Y/2)
                CreateTopTrack(led_din_pos + pcbnew.VECTOR2I_MM(0, -1.25), via_pos, led_din_net, DATA_SIGNAL_TRACK_WIDTH)
                AddSignalVia(via_pos, VIA_SIGNAL_DRILL, VIA_SIGNAL_WIDTH, led_din_net)
                CreateBottomTrack(via_pos, track1_pos, led_din_net, DATA_SIGNAL_TRACK_WIDTH)
            if border_left and led_row != ROWS-1 :
                via_pos = led_dout_pos + pcbnew.VECTOR2I_MM(0, 1.75)
                track1_pos = via_pos + pcbnew.VECTOR2I_MM(1.6, 1.6)
                track2_pos = track1_pos + pcbnew.VECTOR2I_MM(0, LEDS_DISTANCE_Y/2 - 2)

                CreateTopTrack(led_dout_pos, via_pos, led_dout_net, DATA_SIGNAL_TRACK_WIDTH)
                AddSignalVia(via_pos, VIA_SIGNAL_DRILL, VIA_SIGNAL_WIDTH, led_dout_net)
                CreateBottomTrack(via_pos, track1_pos, led_dout_net, DATA_SIGNAL_TRACK_WIDTH)
                CreateBottomTrack(track1_pos, track2_pos, led_dout_net, DATA_SIGNAL_TRACK_WIDTH)

                via_pos = led_din_pos + pcbnew.VECTOR2I_MM(0, -1.25)
                track1_pos = via_pos + pcbnew.VECTOR2I_MM(0, LEDS_DISTANCE_Y/2)

                AddSignalVia(via_pos, VIA_SIGNAL_DRILL, VIA_SIGNAL_WIDTH, led_din_net)
                CreateBottomTrack(via_pos, track1_pos, led_din_net, DATA_SIGNAL_TRACK_WIDTH)

def AdjustSilkScreen():
    for itm in leds:
        cap = itm["cap"]
        cap_ref = cap.Reference()

        cap_num = int(cap.GetReference()[1:]) - 1
        cap_row = int(cap_num / COLS)
        cap_col = cap_num % COLS

        cap_pos = cap_ref.GetPosition()

        if cap_row % 2 == 0:
            cap_pos = cap_pos + pcbnew.VECTOR2I_MM(5, -1)
            cap_ref.SetPosition(cap_pos)
            cap_ref.SetTextAngleDegrees(90)
        else:
            cap_pos = cap_pos + pcbnew.VECTOR2I_MM(-5, 1)
            cap_ref.SetPosition(cap_pos)
            cap_ref.SetTextAngleDegrees(90)

    for itm in pads:
        pad = itm["GND"]
        pad_ref = pad.Reference()

        pad_num = int(pad.GetReference()[3:]) - 1
        pad_pos = pad.GetPosition()

        if pad_num % 2 == 1:
            pad_pos = pad_pos + pcbnew.VECTOR2I_MM(0, 3.15)
            pad_ref.SetPosition(pad_pos)

def PlaceTopPowerPlanes():
    x = BOARD_START + POWER_PLANE_DISTANCE_BOARD
    width = LEDS_DISTANCE_X * COLS - POWER_PLANE_DISTANCE_BOARD * 2

    # create top first power GND plane of half size
    y = BOARD_START + POWER_PLANE_DISTANCE_BOARD
    height = LEDS_DISTANCE_Y/2 - 1.7 - POWER_PLANE_DISTANCE_BOARD
    CreateGndTopPowerPlane(x, y, width, height)

    for i in range(1, ROWS):
        y = BOARD_START + (LEDS_DISTANCE_Y/2 + 1.7) + (i-1)*LEDS_DISTANCE_Y
        height = LEDS_DISTANCE_Y - 1.7*2

        if i % 2 == 0:
            CreateGndTopPowerPlane(x, y, width, height)
        else:
            CreateVddTopPowerPlane(x, y, width, height)

    # create last bottom power GND plane of half size
    y = BOARD_START + ROWS*LEDS_DISTANCE_Y - LEDS_DISTANCE_Y/2 + 1.7
    height = LEDS_DISTANCE_Y/2 - 1.7 - POWER_PLANE_DISTANCE_BOARD
    CreateGndTopPowerPlane(x, y, width, height)

def PlaceBottomInterconnectPlanes():
    y = BOARD_START + POWER_PLANE_DISTANCE_BOARD
    height = ROWS * LEDS_DISTANCE_Y - POWER_PLANE_DISTANCE_BOARD * 2
    width = LEDS_DISTANCE_X * 2 - POWER_PLANE_DISTANCE_BOARD * 2

    for i in range(1, int(COLS/2)):
        x = BOARD_START + LEDS_DISTANCE_X * i * 2 - LEDS_DISTANCE_X + POWER_PLANE_DISTANCE_BOARD
        if i % 2 == 0:
            CreateGndBottomPowerPlane(x, y, width, height)
        else:
            CreateVddBottomPowerPlane(x, y, width, height)

def PlaceViasBetweenPowerPlanes():
    for col in range(0, COLS):
        if col == 0 or col == COLS-1:
            continue

        for row in range(0, ROWS):
            col_m = (col - 1) % 4
            if row % 2 == 0:
                if col_m == 0 or col_m == 1:
                    place_up = False
                    place_gnd = False
                else:
                    place_up = True
                    place_gnd = True
            else:
                if col_m == 0 or col_m == 1:
                    place_up = True
                    place_gnd = False
                else:
                    place_up = False
                    place_gnd = True

            off_x = LEDS_DISTANCE_X / (VIAS_PER_CELL_TO_BOTTOM + 1)
            off_y = LEDS_DISTANCE_Y / 6
            x = BOARD_START + col * LEDS_DISTANCE_X
            y = BOARD_START + row * LEDS_DISTANCE_Y

            if place_up:
                y = y + off_y * 1
            else:
                y = y + off_y * 5

            print("# row: {0}, col: {1}".format(row, col))
            print("place_up: {0}".format(place_up))
            print("place_gnd: {0}".format(place_gnd))

            for v in range(VIAS_PER_CELL_TO_BOTTOM):
                print("x: {0}, y: {1}".format(x + off_x * (v + 1), y))
                if place_gnd:
                    AddGndVia(pcbnew.VECTOR2I_MM(x + off_x * (v + 1), y), VIA_POWERPLANE_DRILL, VIA_POWERPLANE_WIDTH)
                else:
                    AddVddVia(pcbnew.VECTOR2I_MM(x + off_x * (v + 1), y), VIA_POWERPLANE_DRILL, VIA_POWERPLANE_WIDTH)

def GetNetOfPadByName(footprint, pad_name):
    for pad in footprint.Pads():
        if pad.GetPadName() == pad_name:
            return pad.GetNet()

def GetPadByName(footprint, pad_name):
    for pad in footprint.Pads():
        if pad.GetPadName() == pad_name:
         return pad
    return None

def GetPadByNet(footprint, net):
    for pad in footprint.Pads():
        if pad.GetNet().GetNetname() == net.GetNetname():
            return pad
    return None

def PlacePowerPads():
    for pad in pads:
        padvdd = pad["VDD"]
        padvdd_ref = padvdd.GetReference()
        padvdd_pos = padvdd.GetPosition()

        padgnd = pad["GND"]
        padgnd_ref = padgnd.GetReference()
        padgnd_pos = padgnd.GetPosition()

        padvdd.SetZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
        padgnd.SetZoneConnection(pcbnew.ZONE_CONNECTION_FULL)

        pad_num = int(padvdd_ref[3:]) - 1

        if pad_num < ROWS:
            pad_x = BOARD_START
            pad_y = BOARD_START + LEDS_DISTANCE_Y/2 + pad_num*LEDS_DISTANCE_Y
        else:
            pad_x = BOARD_START + (COLS*LEDS_DISTANCE_X) - POWERPAD_OFF_X * 2
            pad_y = BOARD_START + LEDS_DISTANCE_Y/2 + (pad_num-ROWS)*LEDS_DISTANCE_Y

        if pad_num % 2 == 0:
            padgnd.SetPosition(pcbnew.VECTOR2I_MM(pad_x + POWERPAD_OFF_X, pad_y - POWERPAD_OFF_Y))
            padvdd.SetPosition(pcbnew.VECTOR2I_MM(pad_x + POWERPAD_OFF_X, pad_y + POWERPAD_OFF_Y))
        else:
            padvdd.SetPosition(pcbnew.VECTOR2I_MM(pad_x + POWERPAD_OFF_X, pad_y - POWERPAD_OFF_Y))
            padgnd.SetPosition(pcbnew.VECTOR2I_MM(pad_x + POWERPAD_OFF_X, pad_y + POWERPAD_OFF_Y))

def DrawGridSilkScreen():
    for line in range(ROWS - 1):
        x1 = BOARD_START + 1
        y1 = BOARD_START + LEDS_DISTANCE_Y * (line + 1)
        x2 = BOARD_START + COLS *  LEDS_DISTANCE_X - 1
        y2 = BOARD_START + LEDS_DISTANCE_Y * (line + 1)
        CreateLineSilkScreen(x1, y1, x2, y2)

    for column in range(COLS - 1):
        x1 = BOARD_START + LEDS_DISTANCE_X * (column + 1)
        y1 = BOARD_START + 1
        x2 = BOARD_START + LEDS_DISTANCE_X * (column + 1)
        y2 = BOARD_START + ROWS * LEDS_DISTANCE_Y - 1
        CreateLineSilkScreen(x1, y1, x2, y2)

def CleanupBoard():
    pcb = pcbnew.GetBoard()

    for track in pcb.GetTracks():
        pcb.Remove(track)

    for drawing in pcb.GetDrawings():
        pcb.Remove(drawing)

    pcb.Zones().clear()

def FillAllPowerPlanes():
    pcb = pcbnew.GetBoard()
    filler = pcbnew.ZONE_FILLER(pcb)
    filler.Fill(pcb.Zones())

def CreateExclusionZonesForScrews():
    for row in range(1, ROWS):
        x = BOARD_START
        y = BOARD_START + row * LEDS_DISTANCE_Y - HEIGHT_SCREW_CUTOFF / 2
        width = LEDS_DISTANCE_X
        height = HEIGHT_SCREW_CUTOFF
        CreateExclusionZone(x, y, width, height)

        x = BOARD_START + (COLS -1) * LEDS_DISTANCE_X
        y = BOARD_START + row * LEDS_DISTANCE_Y - HEIGHT_SCREW_CUTOFF / 2
        CreateExclusionZone(x, y, width, height)

def CreateExclusionZonesForGrid():
    for row in range(1, ROWS):
        for col in range(1, COLS):
            x = BOARD_START + col * LEDS_DISTANCE_X - HEIGHT_GRID_CUTOFF/2
            y = BOARD_START + row * LEDS_DISTANCE_Y - HEIGHT_GRID_CUTOFF/2
            CreateExclusionZone(x, y, HEIGHT_GRID_CUTOFF, HEIGHT_GRID_CUTOFF)

def PlaceComponents():
    CleanupBoard()

    PopulateComponents()
    PlaceLEDsAndCaps()

    CreateBoardOutline(BOARD_OFF, BOARD_OFF,
                       COLS * LEDS_DISTANCE_X + BOARD_MARGIN,
                       ROWS * LEDS_DISTANCE_Y + BOARD_MARGIN)

    PlaceTopPowerPlanes()

    PlaceBottomInterconnectPlanes()

    PlaceViasBetweenPowerPlanes()

    PlacePowerPads()

    CreateExclusionZonesForGrid()
    CreateExclusionZonesForScrews()

    AdjustSilkScreen()

    DrawGridSilkScreen()

    FillAllPowerPlanes()

    pcbnew.Refresh()

class LedianPlugin(pcbnew.ActionPlugin):
    def defaults(self):
        self.name = "Ledian"
        self.category = "Ledian"
        self.description = "Ledian"
        self.show_toolbar_button = True
        self.here_path, self.filename = os.path.split(os.path.abspath(__file__))
        self.icon_file_name = os.path.join(self.here_path, 'icon_24x24.png')

    def Run(self):
        PlaceComponents()
        wx.MessageBox("Ledian", "Done", wx.OK)

LedianPlugin().register()
