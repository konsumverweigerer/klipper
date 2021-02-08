# Code for supporting mixing extruders.
#
# Copyright (C) 2021 Peter Gruber <nokos@gmx.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import stepper, chelper

from toolhead import Move
from gcode import GCodeCommand

class MixingMove:
    def __init__(self, x, y, z, e,
                 dist_x, dist_y, dist_z, dist_e,
                 ratio_x, ratio_y, ratio_z, ratio_e,
                 distance, acceleration,
                 velocity_start, velocity_cruise,
                 time_accel, time_cruise, time_decel):
        self.axes_d = (dist_x, dist_y, dist_z, dist_e)
        self.axes_r = (ratio_x, ratio_y, ratio_z, ratio_e)
        self.move_d = distance
        self.accel = acceleration
        self.start_v, self.cruise_v = velocity_start, velocity_cruise
        self.accel_t, self.cruise_t, self.decel_t = time_accel, time_cruise, time_decel
        self.start_pos = (x, y, z, e)

class MixingExtruder:
    def __init__(self, config, idx):
        self.printer = config.get_printer()
        self.name = config.get_name()
        extruders = [e.strip() for e in config.get('extruders', None).split(",")]
        if not len(extruders):
            raise self._mcu.get_printer().config_error(
                "No extruders configured for mixing")
        self.main_extruder = extruders[0]
        self.mixing_extruders = {} if idx == 0 else self.printer.lookup_object("mixingextruder").mixing_extruders
        self.mixing_extruders[idx] = self
        pheaters = self.printer.load_object(config, 'heaters')
        try:
            self.heater = pheaters.lookup_heater(self.main_extruder)
        except Exception as e:
            logging.error("no heaters found: %s" % (str(pheaters.get_all_heaters())), e)
        self.extruders = [self.printer.lookup_object(extruder)
                          for extruder in extruders]
        self.mixing = [0 if p else 1 for p in range(len(self.extruders))]
        self.positions = [0. for p in range(len(self.extruders))]
        self.ratios = [0 for p in range(len(self.extruders))]
        logging.info("MixingExtruder extruders=%s", ", ".join(self.extruders))
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        toolhead = self.printer.lookup_object('toolhead')
        if self.name == 'mixingextruder':
            toolhead.set_extruder(self, 0.)
            gcode.register_command("M163", self.cmd_M163)
            gcode.register_command("M164", self.cmd_M164)
            gcode.register_command("M567", self.cmd_M567)
            self.orig_G1 = gcode.register_command("G1", None)
            gcode.register_command("G1", self.cmd_G1)
        gcode.register_mux_command("ACTIVATE_EXTRUDER", "EXTRUDER",
                                   self.name, self.cmd_ACTIVATE_EXTRUDER,
                                   desc=self.cmd_ACTIVATE_EXTRUDER_help)
    def update_move_time(self, flush_time):
        for extruder in self.extruders:
            extruder.update_move_time(flush_time)
    def calc_junction(self, prev_move, move):
        diff_r = move.axes_r[3] - prev_move.axes_r[3]
        if diff_r:
            m = max(self.mixing)
            return (self.instant_corner_v / abs(m * diff_r))**2
        return move.max_cruise_v2
    def _scale_move(self, move, idx):
        mixing = self.mixing[idx]
        if not mixing:
            return None
        return MixingMove(move.start_pos[0], move.start_pos[1], move.start_pos[2], self.positions[idx],
                          move.axes_d[0], move.axes_d[1], move.axes_d[2], mixing * move.axes_d[3],
                          move.axes_r[0], move.axes_r[1], move.axes_r[2], mixing * move.axes_r[3],
                          move.move_d, move.accel, move.start_v, move.cruise_v,
                          move.accel_t, move.cruise_t, move.decel_t)
    def _check_move(self, scaled_move, move):
        axis_r = scaled_move.axes_r[3]
        axis_d = scaled_move.axes_d[3]
        if not self.heater.can_extrude:
            raise self.printer.command_error(
                "Extrude below minimum temp\n"
                "See the 'min_extrude_temp' config option for details")
        if (not move.axes_d[0] and not move.axes_d[1]) or axis_r < 0.:
            # Extrude only move (or retraction move) - limit accel and velocity
            if abs(axis_d) > self.main_extruder.max_e_dist:
                raise self.printer.command_error(
                    "Extrude only move too long (%.3fmm vs %.3fmm)\n"
                    "See the 'max_extrude_only_distance' config"
                    " option for details" % (axis_d, self.max_e_dist))
            inv_extrude_r = 1. / abs(axis_r)
            move.limit_speed(self.main_extruder.max_e_velocity * inv_extrude_r,
                             self.main_extruder.max_e_accel * inv_extrude_r)
        elif axis_r > self.main_extruder.max_extrude_ratio:
            if axis_d <= self.main_extruder.nozzle_diameter * self.main_extruder.max_extrude_ratio:
                # Permit extrusion if amount extruded is tiny
                return
            area = axis_r * self.main_extruder.filament_area
            logging.debug("Overextrude: %s vs %s (area=%.3f dist=%.3f)",
                          axis_r, self.main_extruder.max_extrude_ratio, area, move.move_d)
            raise self.printer.command_error(
                "Move exceeds maximum extrusion (%.3fmm^2 vs %.3fmm^2)\n"
                "See the 'max_extrude_cross_section' config option for details"
                % (area, self.main_extruder.max_extrude_ratio * self.main_extruder.filament_area))
    def check_move(self, move):
        for idx, extruder in enumerate(self.extruders):
            scaled_move = self._scale_move(move, idx)
            if scaled_move:
                self._check_move(scaled_move, move)
    def move(self, print_time, move):
        for idx, extruder in enumerate(self.extruders):
            scaled_move = self._scale_move(move, idx)
            if scaled_move:
                extruder.move(print_time, scaled_move)
                self.positions[idx] = scaled_move.end_pos[3]
    def get_status(self, eventtime):
        return dict(self.stats(eventtime),
                    ticks=", ".join(extruder.stepper.get_mcu_position() for extruder in self.extruders),
                    extruders=", ".join(extruder.name for extruder in self.extruders))
    def get_name(self):
        return self.name
    def get_heater(self):
        return self.heater
    def stats(self, eventtime):
        return self.heater.stats(eventtime)
    def cmd_M163(self, gcmd):
        index = gcmd.get_int('S', None, minval=0, maxval=len(self.extruders))
        weight = gcmd.get_float('P', 0., minval=0.)
        self.ratios[index] = weight
    def cmd_M164(self, gcmd):
        mixingextruder = self
        index = gcmd.get_int('S', 0, minval=0, maxval=len(self.extruders))
        if index:
            mixingextruder = self.printer.lookup_object(
                "mixingextruder%d" % (index))
            if not mixingextruder:
                raise gcmd.error("Invalid extruder index")
        s = sum(self.ratios)
        if s <= 0:
            raise gcmd.error("Could not save ratio: its empty")
        for i, v in enumerate(self.ratios):
            mixingextruder.mixing[i] = v/s
    def cmd_M567(self, gcmd):
        mixingextruder = self
        index = gcmd.get_int('P', 0, minval=0, maxval=len(self.extruders))
        if index:
            mixingextruder = self.printer.lookup_object(
                "mixingextruder%d" % (index))
            if not mixingextruder:
                raise gcmd.error("Invalid extruder index")
        weighting = gcmd.get('E', None)
        if not weighting:
            raise gcmd.error("No ratio in M567")
        weights = [float(w) for w in weighting.split(":")]
        if min(weights) < 0:
            raise gcmd.error("Negative weight not allowed")
        s = sum(weights)
        if not 0 <= s < 1:
            raise gcmd.error("Could not save ratio: its empty")
        for i, v in enumerate(weights):
            mixingextruder.mixing[i] = v/s
    def cmd_G1(self, gcmd):
        gcode = self.printer.lookup_object('gcode')
        weighting = gcmd.get('E', None)
        if not weighting or ":" not in weighting:
            self.orig_G1(gcmd)
        weights = [float(w) for w in weighting.split(":")]
        if min(weights) < 0:
            raise gcmd.error("Negative weight not allowed")
        extrude = sum(weights)
        self.cmd_M567(GCodeCommand(
            gcode, "M567", "M567 E%s" % (weighting),
            dict(E=weighting), gcmd._need_ack))
        self.orig_G1(GCodeCommand(
            gcode, "G1", gcmd.get_commandline(),
            dict(gcmd.get_command_parameters(), E="%f" % (extrude)),
            gcmd._need_ack))
    cmd_ACTIVATE_EXTRUDER_help = "Change the active extruder"
    def cmd_ACTIVATE_EXTRUDER(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead.get_extruder() is self:
            gcmd.respond_info("Extruder %s already active" % (self.name,))
            return
        gcmd.respond_info("Activating extruder %s" % (self.name,))
        toolhead.flush_step_generation()
        toolhead.set_extruder(self, self.stepper.get_commanded_position())
        self.printer.send_event("extruder:activate_extruder")

def load_config(config):
    printer = config.get_printer()
    for i in range(16):
        section = 'mixingextruder'
        if i:
            section = 'mixingextruder%d' % (i,)
        pe = MixingExtruder(config.getsection('mixingextruder'), i)
        printer.add_object(section, pe)