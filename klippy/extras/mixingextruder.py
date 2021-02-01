# Code for supporting mixing extruders.
#
# Copyright (C) 2021 Peter Gruber <nokos@gmx.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import stepper, chelper

class MixingExtruder:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        extruders = config.get('extruders', None)
        if not len(extruders):
            logging.error("No extruders configured for mixing")
            return
        self.main_extruder = extruders[0]
        pheaters = self.printer.load_object(config, 'heaters')
        self.heater = pheaters.lookup_heater(self.main_extruder)
        self.extruders = [self.printer.lookup_object(extruder) for extruder in extruders]
        self.mixing = [0 if p else 1 for p in range(len(self.extruders))]
        self.ratios = [0 for p in range(len(self.extruders))]
        logging.info("MixingExtruder extruders=%s", ", ".join(self.extruders))
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        if self.name == 'mixingextruder':
            gcode.register_command("M163", self.cmd_M163)
            gcode.register_command("M164", self.cmd_M164)
            gcode.register_command("M567", self.cmd_M567)
        gcode.register_mux_command("ACTIVATE_EXTRUDER", "EXTRUDER",
                                   self.name, self.cmd_ACTIVATE_EXTRUDER,
                                   desc=self.cmd_ACTIVATE_EXTRUDER_help)
    def update_move_time(self, flush_time):
        for extruder in seld.extruders:
            extruder.update_move_time(flush_time)
    def check_move(self, move):
        # TODO
        raise move.move_error("Extrude when no extruder present")
    def calc_junction(self, prev_move, move):
        return self.main_extruder.calc_junction(prev_move, move)
    def move(self, print_time, move):
        # TODO
        pass
    def get_status(self, eventtime):
        return dict(self.stats(eventtime),
                    extruders=self.extruders)
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
        index = gcmd.get_int('P', None, minval=0, maxval=len(self.extruders))
        weighting = gcmd.get('E', None)
        s = sum(weighting)
        if not 0 <= s < 1:
            raise gcmd.error("Could not save ratio: its empty")
        for i, v in enumerate(weightig):
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
