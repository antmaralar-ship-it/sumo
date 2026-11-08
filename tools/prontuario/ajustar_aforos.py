#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
# Copyright (C) 2026-2026 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    ajustar_aforos.py
# @date    2026-11-08

"""
Ajusta la demanda de coches de un escenario a los aforos de campo
introducidos en sumo-gui (aforos_campo.xml).

1. Calcula las rutas de los viajes de coche del escenario (duarouter).
2. Con routeSampler conserva esos viajes como punto de partida, quita los
   que sobran en las calles aforadas y añade los que faltan.
3. Cambia el .sumocfg para usar la nueva demanda (guarda una copia
   osm.sumocfg.antes_aforos la primera vez).
"""

from __future__ import print_function
import argparse
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import sumolib  # noqa

OUT_ROUTES = "prontuario_coches_aforos.rou.xml"
CANDIDATES = "prontuario_candidatos.rou.xml"


def cfg_value(root, name):
    for el in root.iter(name):
        return el.get("value")
    return None


def set_cfg_value(root, name, value):
    for el in root.iter(name):
        el.set("value", value)
        return


def is_car_file(name):
    base = os.path.basename(name).lower()
    return ("passenger" in base or "coche" in base) and OUT_ROUTES not in base


def main(args):
    op = argparse.ArgumentParser(description="Ajusta la demanda de coches a los aforos de campo")
    op.add_argument("-c", "--config", required=True, help="archivo .sumocfg del escenario")
    op.add_argument("-a", "--aforos", help="archivo de aforos (por defecto aforos_campo.xml junto al .sumocfg)")
    options = op.parse_args(args)

    cfg = os.path.abspath(options.config)
    folder = os.path.dirname(cfg)
    aforos = options.aforos or os.path.join(folder, "aforos_campo.xml")
    if not os.path.isfile(aforos):
        print("ERROR: no hay aforos de campo (%s)." % aforos)
        return 1
    backup = cfg + ".antes_aforos"
    source_cfg = backup if os.path.isfile(backup) else cfg
    root = ET.parse(source_cfg).getroot()
    net = cfg_value(root, "net-file")
    routes = cfg_value(root, "route-files") or ""
    begin = cfg_value(root, "begin") or "0"
    end = cfg_value(root, "end") or "3600"
    route_list = [r.strip() for r in routes.split(",") if r.strip()]
    car_files = [r for r in route_list if is_car_file(r)]
    if not car_files:
        print("ERROR: no encuentro el archivo de viajes de coches en route-files (%s)." % routes)
        print("El nombre debe contener 'passenger' (como en osmWebWizard) o 'coche'.")
        return 1
    print("Viajes de coche de partida: %s" % ", ".join(car_files))

    # 1. rutas candidatas
    duarouter = sumolib.checkBinary("duarouter")
    cmd = [duarouter, "-n", os.path.join(folder, net), "-r", ",".join(os.path.join(folder, f) for f in car_files),
           "-o", os.path.join(folder, CANDIDATES), "--ignore-errors", "--no-step-log", "--no-warnings",
           "--begin", begin, "--end", end]
    print("Calculando rutas (duarouter)...")
    sys.stdout.flush()
    subprocess.check_call(cmd)

    # routeSampler necesita numpy
    try:
        import numpy  # noqa
    except ImportError:
        print("Instalando numpy, que necesita routeSampler (solo la primera vez)...")
        sys.stdout.flush()
        subprocess.call([sys.executable, "-m", "pip", "install", "--user", "numpy"])

    # 2. routeSampler conservando la demanda inicial
    sampler = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "routeSampler.py")
    cmd = [sys.executable, sampler, "-r", os.path.join(folder, CANDIDATES), "-d", aforos,
           "-o", os.path.join(folder, OUT_ROUTES), "--init-input.remove-overflow", "--keep-attributes",
           "--edgedata-attribute", "entered", "-b", begin, "-e", end,
           "--mismatch-output", os.path.join(folder, "prontuario_aforos_desajuste.xml"), "-v"]
    print("Ajustando la demanda a los aforos (routeSampler)...")
    sys.stdout.flush()
    output = subprocess.check_output(cmd, universal_newlines=True)
    print(output)

    # 3. usar la nueva demanda en el escenario
    if not os.path.isfile(backup):
        shutil.copyfile(cfg, backup)
        print("Copia del escenario original: %s" % os.path.basename(backup))
    new_routes = [r for r in route_list if not is_car_file(r) and OUT_ROUTES not in r] + [OUT_ROUTES]
    root_cfg = ET.parse(cfg)
    set_cfg_value(root_cfg.getroot(), "route-files", ",".join(new_routes))
    root_cfg.write(cfg, encoding="UTF-8", xml_declaration=True)
    print("El escenario usa ahora %s en lugar de %s." % (OUT_ROUTES, ", ".join(car_files)))
    print("Para volver a la demanda original, copia %s sobre %s." % (os.path.basename(backup), os.path.basename(cfg)))
    gehs = re.findall(r"GEH.*", output)
    if gehs:
        print("Resumen: " + gehs[-1])
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv[1:]))
    except subprocess.CalledProcessError as e:
        print("ERROR al ejecutar: %s" % " ".join(e.cmd))
        sys.exit(1)
