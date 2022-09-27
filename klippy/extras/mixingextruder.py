# Code for supporting mixing extruders.
#
# Copyright (C) 2021 Peter Gruber <nokos@gmx.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import logging


def extruder_to_idx(name, active=None):
    name = name.lower()
    if name == "active":
        name = active() if active else ''
    if name.startswith('mixingextruder'):
        if name[14:] == '':
            return 0
        try:
            return int(name[14:])
        except Exception:
            pass
    try:
        return int(name)
    except Exception:
        return -1


def idx_to_extruder(idx):
    return "mixingextruder%d" % (idx) if idx else "mixingextruder"


class MixingExtruder:
    def __init__(self, config, idx, parent=None):
        self.printer = config.get_printer()
        self.activated = False
        self.name = idx_to_extruder(idx)
        self.stepper_names = [e.strip()
                               for e in
                               config.get('steppers', '').split(",")
                               if len(e)]
        if not len(self.stepper_names):
            raise self.printer.config_error(
                "No steppers configured for mixing")
        self.steppers = parent.steppers if parent else []
        self.mixing_extruders = parent.mixing_extruders if parent else {}
        self.mixing_extruders[idx] = self
        self.mixing = self._init_mixings(idx, len(self.stepper_names))
        # ratios are used in SAVE_MIXING_EXTRUDERS
        self.ratios = [1 if p == 0 else 0 for p in range(len(self.stepper_names))]
        self.enabled = False
        self.gradient_enabled = False
        # assumed to be sorted list of ((start, middle, end), (ref1, ref2))
        self.gradients = []
        self.gradient_method = 'linear'
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        self.printer.register_event_handler('toolhead:move',
                                            self._handle_move)
        logging.info("MixingExtruder %d steppers=%s mixing %s", idx,
                     ",".join(self.stepper_names),
                     ",".join("%.1f" % (x) for x in self.mixing))
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("ACTIVATE_EXTRUDER", "EXTRUDER",
                                   self.name, self.cmd_ACTIVATE_EXTRUDER,
                                   desc=self.cmd_ACTIVATE_EXTRUDER_help)
        if not idx:
            gcode.register_command("SET_MIXING_EXTRUDER",
                                   self.cmd_SET_MIXING_EXTRUDER,
                                   desc=self.cmd_SET_MIXING_EXTRUDER_help)
            gcode.register_command("SAVE_MIXING_EXTRUDERS",
                                   self.cmd_SAVE_MIXING_EXTRUDERS,
                                   desc=self.cmd_SAVE_MIXING_EXTRUDERS_help)
        gcode.register_mux_command("ADD_MIXING_GRADIENT", "EXTRUDER",
                                   self.name, self.cmd_ADD_MIXING_GRADIENT,
                                   desc=self.cmd_ADD_MIXING_GRADIENT_help)
        gcode.register_mux_command("RESET_MIXING_GRADIENT", "EXTRUDER",
                                   self.name, self.cmd_RESET_MIXING_GRADIENT,
                                   desc=self.cmd_RESET_MIXING_GRADIENT_help)
        gcode.register_mux_command("SET_MIXING_GRADIENT", "EXTRUDER",
                                   self.name, self.cmd_SET_MIXING_GRADIENT,
                                   desc=self.cmd_SET_MIXING_GRADIENT_help)
        gcode.register_mux_command("MIXING_STATUS", "EXTRUDER",
                                   self.name, self.cmd_MIXING_STATUS,
                                   desc=self.cmd_MIXING_STATUS_help)

    def _init_mixings(self, idx, steppers_count):
        if idx == 0:
            return [1. / steppers_count for p in range(steppers_count)]
        idx = idx - 1
        if idx < steppers_count:
            return [1. if p == idx else 0. for p in range(steppers_count)]
        idx = idx - steppers_count
        if idx < steppers_count:
            return [0. if p == idx else 1. / (steppers_count - 1)
                    for p in range(steppers_count)]
        idx = idx - steppers_count
        if steppers_count == 3:
            if idx < 2 * steppers_count:
                return [[0. if p == x else (1 + (((p - x) % 3 + y) % 2)) / 3.
                         for p in range(steppers_count)]
                        for x in range(steppers_count) for y in (0, 1)][idx]
            idx = idx - 2 * steppers_count
        elif steppers_count > 3:
            if idx < (steppers_count * (steppers_count - 1) / 2):
                return [[0. if p == x or p == y else 1. / (steppers_count - 2)
                         for p in range(steppers_count)]
                        for x in range(steppers_count)
                        for y in range(x + 1, steppers_count)][idx]
            idx = idx - (steppers_count * (steppers_count - 1) / 2)
        return [1. / steppers_count for p in range(steppers_count)]

    def _handle_connect(self):
        if self.activated:
            return
        self.activated = True
        if self.mixing_extruders[0] != self:
            return
        try:
            for name in self.stepper_names:
                try:
                    stepper = self.printer.lookup_object(name).extruder_stepper
                except:
                    stepper = self.printer.lookup_object("extruder_stepper_" + name)
                self.steppers.append(stepper)
        except Exception as e:
            while len(self.steppers) > 0:
                self.steppers.pop()
            logging.error("no steppers found: %s: %s",
                          ", ".join(self.stepper_names), e)

    def _handle_move(self):
        if self.enabled and self.gradient_enabled:
            toolhead = self.printer.lookup_object('toolhead')
            self._apply_mixing(toolhead.get_position())

    def _get_gradient(self, pos):
        default = self.mixing
        for heights, refs in self.gradients:
            start, _, end = heights
            start_mix, end_mix = (self.mixing_extruders[i].mixing
                                  for i in refs)
            if self.gradient_method == 'linear':
                zpos = pos[2]
                if zpos <= start:
                    return start_mix
                if zpos >= end:
                    default = end_mix
                    continue
                w = (zpos - start) / (end - start)
                logging.info("linear gradient @%.1f(%.1f-%.1f) [%s-%s]" %
                             (zpos, start, end,
                              "/".join("%.1f" % x for x in start_mix),
                              "/".join("%.1f" % x for x in end_mix)))
                return list(((1. - w) * s + w * e)
                            for s, e in zip(start_mix, end_mix))
            if self.gradient_method == 'spherical':
                logging.info("spherical gradient at (%s)",
                             ", ".join("%.1f" % x for x in pos))
                dist = math.sqrt(sum(x**2 for x in pos))
                if dist <= start:
                    return start_mix
                if dist >= end:
                    default = end_mix
                    continue
                w = (dist - start) / (end - start)
                mix = list(((1. - w) * s + w * e)
                           for s, e in zip(start_mix, end_mix))
                logging.info("spherical gradient @%.1f(%.1f-%.1f) [%s-%s]=%s" %
                             (dist, start, end,
                              "/".join("%.1f" % x for x in start_mix),
                              "/".join("%.1f" % x for x in end_mix),
                              "/".join("%.1f" % x for x in mix)))
                return mix
        return default

    def get_status(self, eventtime):
        status = dict(mixing=",".join("%0.1f%%" % (m * 100.)
                                      for m in self.mixing),
                      mixing_ticks=",".join("%0.2f" % (
                                            stepper.stepper.get_mcu_position())
                                            for stepper in self.steppers),
                      mixing_extruders=",".join(stepper.name
                                                for stepper in self.steppers))
        for i, gradient in enumerate(self.gradients):
            status.update({"mixing_gradient%d" % (i): ",".join(
                "%s:%s" % (k, v)
                for k, v in dict(
                    heights="%.1f-(%.1f)-%.1f" % gradient[0],
                    mixings="%s-%s" % tuple(
                        "/".join("%.1f" % (x)
                                 for x in self.mixing_extruders[i].mixing)
                        for i in gradient[1]),
                    method=self.gradient_method,
                    enabled=str(self.gradient_enabled)).items())})
        return status

    def get_name(self):
        return self.name

    def get_heater(self):
        return self.steppers[0].extruder.get_heater()

    def _apply_mixing(self, position=None):
        mix = self.mixing
        if self.gradient_enabled and position:
            mix = self._get_gradient(position)
        extruder_name = self.steppers[0].extruder_name
        for i, stepper in enumerate(self.steppers):
            if extruder_name != stepper.extruder_name:
                stepper.sync_to_extruder(extruder_name)
            stepper.set_extrusion_factor(mix[i])

    cmd_SET_MIXING_EXTRUDER_help = "Set scale on motor/extruder"

    def cmd_SET_MIXING_EXTRUDER(self, gcmd):
        name = gcmd.get('STEPPER')
        scale = gcmd.get_float('SCALE', minval=0.)
        if name not in self.stepper_names:
            try:
                index = int(name)
                if not 0 <= index < len(self.stepper_names):
                    raise Exception("Invalid index")
            except Exception as e:
                raise gcmd.error("Invalid stepper/motor: %s" % (e.message))
        else:
            index = self.stepper_names.index(name)
        self.ratios[index] = scale

    cmd_SAVE_MIXING_EXTRUDERS_help = "Save the scales on motors"

    def cmd_SAVE_MIXING_EXTRUDERS(self, gcmd):
        mixingextruder = self
        extruder = gcmd.get('MIXING_EXTRUDER', None)
        if extruder:
            idx = self._to_idx(extruder)
            if idx >= 0:
                mixingextruder = self.mixing_extruders[idx]
        s = sum(self.ratios)
        if s <= 0:
            raise gcmd.error("Could not save ratio: its empty")
        for i, v in enumerate(self.ratios):
            mixingextruder.mixing[i] = v / s
            self.ratios[i] = 0.0

    cmd_SET_MIXING_GRADIENT_help = "Turn no/off grdient mixing"

    def cmd_SET_MIXING_GRADIENT(self, gcmd):
        method = gcmd.get('METHOD')
        if method in ["linear", "spherical"]:
            self.gradient_method = method
        try:
            enable = gcmd.get_int('ENABLE', 1, minval=0, maxval=1)
            self.gradient_enabled = enable == 1
        except Exception:
            enable = gcmd.get('ENABLE', '')
            self.gradient_enabled = enable.lower() == 'true'

    cmd_ADD_MIXING_GRADIENT_help = "Add mixing gradient"

    def _active_extruder(self):
        toolhead = self.printer.lookup_object('toolhead')
        return toolhead.get_extruder().get_name().lower()

    def _to_idx(self, name):
        return extruder_to_idx(name, active=self._active_extruder)

    def cmd_ADD_MIXING_GRADIENT(self, gcmd):
        start_extruder = self._to_idx(gcmd.get('START'))
        end_extruder = self._to_idx(gcmd.get('END'))
        if start_extruder not in self.mixing_extruders.keys() or \
                end_extruder not in self.mixing_extruders.keys():
            raise gcmd.error("Invalid start/end value")
        start_height = gcmd.get_float('START_HEIGHT', minval=0.)
        end_height = gcmd.get_float('END_HEIGHT', minval=0.)
        if start_height > end_height:
            start_height, end_height = end_height, start_height
            start_extruder, end_extruder = end_extruder, start_extruder
        for gradient in self.gradients:
            s, _, e = gradient[0]
            if e > start_height and end_height > s:
                raise gcmd.error(
                    "Could not configure gradient: overlapping starts/ends")
        self.gradients.append((
            (start_height,
             (start_height + end_height) / 2,
             end_height),
            (start_extruder, end_extruder)))
        self.gradients.sort(key=lambda x: x[0][0])

    cmd_RESET_MIXING_GRADIENT_help = "Clear mixing gradient info"

    def cmd_RESET_MIXING_GRADIENT(self, gcmd):
        self.gradient_enabled, self.gradients, self.gradient_method = \
            False, [], 'linear'

    cmd_ACTIVATE_EXTRUDER_help = "Change the active extruder"

    def cmd_ACTIVATE_EXTRUDER(self, gcmd):
        if self.enabled:
            gcmd.respond_info("Extruder %s already active" % (self.name,))
            return
        for extruder in self.mixing_extruders.values():
            extruder.enabled = (extruder == self)
        gcmd.respond_info("Activating extruder %s" % (self.name,))
        toolhead = self.printer.lookup_object('toolhead')
        self._apply_mixing(toolhead.get_position())

    cmd_MIXING_STATUS_help = "Display the status of the given MixingExtruder"

    def cmd_MIXING_STATUS(self, gcmd):
        eventtime = self.printer.get_reactor().monotonic()
        status = self.get_status(eventtime)
        gcmd.respond_info(", ".join("%s=%s" % (k, v)
                                    for k, v in status.items()))


def load_config(config):
    mixingextruder = None
    for i in range(16):
        pe = MixingExtruder(config.getsection('mixingextruder'),
                            i, parent=mixingextruder)
        if i == 0:
            mixingextruder = pe
    logging.info("Started mixingextruder")
    return mixingextruder
