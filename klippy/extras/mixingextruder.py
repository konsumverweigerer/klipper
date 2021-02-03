# Code for supporting mixing extruders.
#
# Copyright (C) 2021 Peter Gruber <nokos@gmx.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import stepper, chelper

from toolhead import Move

class MixingExtruder:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        extruders = config.get('extruders', None)
        if not len(extruders):
            raise self._mcu.get_printer().config_error(
                "No extruders configured for mixing")
        self.main_extruder = extruders[0]
        pheaters = self.printer.load_object(config, 'heaters')
        self.heater = pheaters.lookup_heater(self.main_extruder)
        self.extruders = [self.printer.lookup_object(extruder) for extruder in extruders]
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
        extrude_start_pos = self.positions[idx]
        extrude_end_pos = extrude_start_pos + mixing * (move.end_pos[3] - move.start_pos[3])
        # TODO: create reduced MixingMove
        return Move(self.printer.lookup_object('toolhead'),
                    move.start_pos[0:3] + (extrude_start_pos,),
                    move.end_pos[:3] + (extrude_end_pos,),
                    (move.move_d / move.min_move_t) if move.is_kenimatic_move else (mixing * move.move_d / move.min_move_t))
    def _check_move(self, scaled_move, move):
        axis_r = scaled_move.axes_r[3]
        axis_d = scaled_move.axes_d[3]
        if not self.heater.can_extrude:
            raise self.printer.command_error(
                "Extrude below minimum temp\n"
                "See the 'min_extrude_temp' config option for details")
        if (not move.axes_d[0] and not move.axes_d[1]) or axis_r < 0.:
            # Extrude only move (or retraction move) - limit accel and velocity
            if abs(axis_d) > self.max_e_dist:
                raise self.printer.command_error(
                    "Extrude only move too long (%.3fmm vs %.3fmm)\n"
                    "See the 'max_extrude_only_distance' config"
                    " option for details" % (axis_d, self.max_e_dist))
            inv_extrude_r = 1. / abs(axis_r)
            move.limit_speed(self.max_e_velocity * inv_extrude_r,
                             self.max_e_accel * inv_extrude_r)
        elif axis_r > self.max_extrude_ratio:
            if axis_d <= self.nozzle_diameter * self.max_extrude_ratio:
                # Permit extrusion if amount extruded is tiny
                return
            area = axis_r * self.filament_area
            logging.debug("Overextrude: %s vs %s (area=%.3f dist=%.3f)",
                          axis_r, self.max_extrude_ratio, area, move.move_d)
            raise self.printer.command_error(
                "Move exceeds maximum extrusion (%.3fmm^2 vs %.3fmm^2)\n"
                "See the 'max_extrude_cross_section' config option for details"
                % (area, self.max_extrude_ratio * self.filament_area))
    def check_move(self, move):
        for idx, extruder in enumerate(self.extruders):
            scaled_move = self._scale_move(move, idx)
            self._check_move(scaled_move, move)
    def move(self, print_time, move):
        for idx, extruder in enumerate(self.extruders):
            scaled_move = self._scale_move(move, idx)
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
        index = gcmd.get_int('S', None, minval=0, maxval=len(self.extruders))
        s = sum(self.ratios)
        if s <= 0:
            raise gcmd.error("Could not save ratio: its empty")
        for i, v in enumerate(self.ratios):
            self.mixing[i] = v/s
    def cmd_M567(self, gcmd):
        index = gcmd.get_int('P', 0, minval=0, maxval=len(self.extruders))
        weighting = gcmd.get('E', None)
        s = sum(weighting)
        if not 0 <= s < 1:
            raise gcmd.error("Could not save ratio: its empty")
        for i, v in enumerate(weighting):
            self.mixing[i] = v/s
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
