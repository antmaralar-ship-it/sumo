/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2001-2026 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// This Source Code may also be made available under the following Secondary
// Licenses when the conditions for such availability set forth in the Eclipse
// Public License 2.0 are satisfied: GNU General Public License, version 2
// or later which is available at
// https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
/// @file    MSProntuario.cpp
/// @date    November 2026
///
// Street indicators graded with the "Prontuario de urbanismo y movilidad"
/****************************************************************************/
#include <config.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <sstream>
#include <utils/common/StringUtils.h>
#include <utils/common/SUMOVehicleClass.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/StdDefs.h>
#include <utils/geom/PositionVector.h>
#include <utils/geom/GeomHelper.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/options/OptionsCont.h>
#include <microsim/MSNet.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSLink.h>
#include <microsim/MSJunction.h>
#include <microsim/MSVehicleControl.h>
#include <microsim/MSVehicleType.h>
#include <microsim/traffic_lights/MSTrafficLightLogic.h>
#include <microsim/traffic_lights/MSPhaseDefinition.h>
#include "MSDetectorControl.h"
#include "MSMeanData_Net.h"
#include "MSMeanData_Emissions.h"
#include "MSMeanData_Harmonoise.h"
#include "MSProntuario.h"


// ===========================================================================
// static members
// ===========================================================================
MSNet* MSProntuario::myNet = nullptr;
MSMeanData* MSProntuario::myMotor = nullptr;
MSMeanData* MSProntuario::myCars = nullptr;
MSMeanData* MSProntuario::myBikes = nullptr;
MSMeanData* MSProntuario::myBuses = nullptr;
MSMeanData* MSProntuario::myPeds = nullptr;
MSMeanData* MSProntuario::myEmissions = nullptr;
MSMeanData* MSProntuario::myNoise = nullptr;
SUMOTime MSProntuario::myBegin = 0;
std::map<std::string, std::vector<const MSEdge*> > MSProntuario::myCrossings;


// ===========================================================================
// helpers
// ===========================================================================
namespace {

/// @brief traffic collector restricted to some vehicle classes
class ProntuarioNet : public MSMeanData_Net {
public:
    ProntuarioNet(const std::string& id, SUMOTime begin, SVCPermissions classes, int detectPersons) :
        MSMeanData_Net(id, begin, SUMOTime_MAX, false, "false", false, false, detectPersons,
                       100000, 0, SUMO_const_haltingSpeed, 0, "", "", std::vector<MSEdge*>(), AggregateType::NO),
        myClasses(classes) {
        if (myClasses != SVCAll) {
            // a non-empty type set enables filtering, real ids are added as they appear
            myVehicleTypes.insert("__prontuario__");
        }
    }

    void detectorUpdate(const SUMOTime step) override {
        refreshTypes();
        MSMeanData_Net::detectorUpdate(step);
    }

    void refreshTypes() {
        if (myClasses == SVCAll) {
            return;
        }
        MSVehicleControl& vc = MSNet::getInstance()->getVehicleControl();
        std::vector<std::string> ids;
        vc.insertVTypeIDs(ids);
        if ((int)ids.size() == myKnownTypes) {
            return;
        }
        myKnownTypes = (int)ids.size();
        for (const std::string& id : ids) {
            if (vc.hasVTypeDistribution(id)) {
                continue;
            }
            MSVehicleType* t = vc.getVType(id, nullptr, true);
            if (t != nullptr && (t->getVehicleClass() & myClasses) != 0) {
                myVehicleTypes.insert(id);
            }
        }
    }

private:
    SVCPermissions myClasses;
    int myKnownTypes = -1;
};


const double NO_DATA = -1e30;

double value(const MSMeanData* md, const MSEdge* e, SumoXMLAttr a) {
    if (md == nullptr || e->getLanes().empty()) {
        return NO_DATA;
    }
    const double v = md->getAttributeValue(e->getLanes().front(), a, NO_DATA);
    return std::isfinite(v) ? v : NO_DATA;
}

bool has(double v) {
    return v > NO_DATA / 2;
}

/// @brief number with decimal comma and thousands dot (Spanish)
std::string num(double v, int dec = 0) {
    if (!has(v) || !std::isfinite(v)) {
        return "-";
    }
    char buf[64];
    snprintf(buf, sizeof(buf), "%.*f", dec, v);
    std::string s(buf);
    std::string sign;
    if (!s.empty() && s[0] == '-') {
        sign = "-";
        s = s.substr(1);
    }
    std::string ip = s, fp;
    const std::string::size_type p = s.find('.');
    if (p != std::string::npos) {
        ip = s.substr(0, p);
        fp = s.substr(p + 1);
    }
    if (ip.size() > 4) {
        std::string g;
        int c = 0;
        for (int i = (int)ip.size() - 1; i >= 0; i--) {
            g.insert(g.begin(), ip[i]);
            if (++c % 3 == 0 && i > 0) {
                g.insert(g.begin(), '.');
            }
        }
        ip = g;
    }
    return sign + ip + (fp.empty() ? "" : "," + fp);
}

std::string hhmm(double s) {
    const int t = (int)std::round(s);
    char buf[32];
    snprintf(buf, sizeof(buf), "%d:%02d", t / 3600, (t % 3600) / 60);
    return buf;
}

enum Grade { OPT, ADEC, INSUF, DATO, NA, ND };

const char* gradeText(Grade g) {
    switch (g) {
        case OPT:
            return "[ÓPTIMO]";
        case ADEC:
            return "[ADECUADO]";
        case INSUF:
            return "[INSUFICIENTE]";
        case DATO:
            return "[dato]";
        case NA:
            return "[no aplica]";
        default:
            return "[sin datos]";
    }
}

/// @brief wrap text to the given width with an indent
void wrap(std::vector<std::string>& out, const std::string& text, const std::string& indent, int width = 92) {
    std::istringstream in(text);
    std::string word, line;
    while (in >> word) {
        if (!line.empty() && (int)(indent.size() + line.size() + 1 + word.size()) > width) {
            out.push_back(indent + line);
            line.clear();
        }
        line += (line.empty() ? "" : " ") + word;
    }
    if (!line.empty()) {
        out.push_back(indent + line);
    }
}

/// @brief one graded indicator
void row(std::vector<std::string>* out, const std::string& name, const std::string& val, Grade g,
         const std::string& umbral = "", const std::string& note = "", const std::string& src = "") {
    if (out == nullptr) {
        return;
    }
    std::string n = name;
    // pad by characters (UTF-8 aware) to align the values
    int len = 0;
    for (unsigned char c : n) {
        if ((c & 0xC0) != 0x80) {
            len++;
        }
    }
    std::string v = val.empty() ? "-" : val;
    int vlen = 0;
    for (unsigned char c : v) {
        if ((c & 0xC0) != 0x80) {
            vlen++;
        }
    }
    const int pad = MAX2(1, 62 - len - vlen);
    out->push_back("  " + n + " " + std::string(pad, '.') + " " + v + "  " + gradeText(g));
    if (!umbral.empty()) {
        wrap(*out, umbral + (src.empty() ? "" : " (" + src + ")"), "      ");
    } else if (!src.empty()) {
        wrap(*out, "(" + src + ")", "      ");
    }
    if (!note.empty()) {
        wrap(*out, "> " + note, "      ");
    }
}

void section(std::vector<std::string>* out, const std::string& title) {
    if (out != nullptr) {
        out->push_back("");
        out->push_back(title);
        out->push_back(std::string(title.size() > 70 ? 70 : title.size(), '-'));
    }
}

std::string compass(const MSEdge* e) {
    const MSLane* l = e->getLanes().front();
    const PositionVector& s = l->getShape();
    if (s.size() < 2) {
        return "";
    }
    const double dx = s.back().x() - s.front().x();
    const double dy = s.back().y() - s.front().y();
    double h = std::atan2(dx, dy) * 180. / M_PI;
    if (h < 0) {
        h += 360;
    }
    static const char* dirs[] = {"N", "NE", "E", "SE", "S", "SO", "O", "NO"};
    return dirs[((int)std::round(h / 45.)) % 8];
}

std::string prettyType(const std::string& t) {
    static const std::map<std::string, std::string> names = {
        {"highway.motorway", "autopista"}, {"highway.trunk", "autovía"}, {"highway.primary", "vía primaria"},
        {"highway.secondary", "vía secundaria"}, {"highway.tertiary", "vía terciaria"},
        {"highway.residential", "calle residencial"}, {"highway.living_street", "calle de convivencia"},
        {"highway.service", "vía de servicio"}, {"highway.unclassified", "vía sin clasificar"},
        {"highway.pedestrian", "calle peatonal"}, {"highway.footway", "senda peatonal"},
        {"highway.cycleway", "carril bici"}
    };
    auto it = names.find(t);
    if (it != names.end()) {
        return it->second;
    }
    if (StringUtils::endsWith(t, "_link")) {
        return "enlace";
    }
    return t.empty() ? "vía" : t;
}

struct TLStat {
    double cycle = 0, green = 0, maxGreen = 0, maxRed = 0;
    bool actuated = false;
};

TLStat tlStats(const MSTrafficLightLogic* tl, int index) {
    TLStat r;
    const MSTrafficLightLogic::Phases& ph = tl->getPhases();
    r.actuated = tl->getLogicType() != TrafficLightType::STATIC;
    if (ph.empty() || index < 0) {
        return r;
    }
    auto isGreen = [&](const MSPhaseDefinition * p) {
        const std::string& st = p->getState();
        return index < (int)st.size() && (st[index] == 'G' || st[index] == 'g');
    };
    for (const MSPhaseDefinition* p : ph) {
        const double d = STEPS2TIME(p->duration);
        r.cycle += d;
        if (isGreen(p)) {
            r.green += d;
        }
    }
    // longest uninterrupted green / red (the cycle repeats)
    double cg = 0, cr = 0;
    for (int i = 0; i < 2 * (int)ph.size(); i++) {
        const MSPhaseDefinition* p = ph[i % ph.size()];
        const double d = STEPS2TIME(p->duration);
        if (isGreen(p)) {
            cg += d;
            cr = 0;
        } else {
            cr += d;
            cg = 0;
        }
        r.maxGreen = MAX2(r.maxGreen, MIN2(cg, r.cycle));
        r.maxRed = MAX2(r.maxRed, MIN2(cr, r.cycle));
    }
    return r;
}

bool isCarLane(const MSLane* l) {
    return l->allowsVehicleClass(SVC_PASSENGER);
}

bool isBusOnly(const MSLane* l) {
    return !l->allowsVehicleClass(SVC_PASSENGER) && l->allowsVehicleClass(SVC_BUS);
}

bool isSidewalk(const MSLane* l) {
    return l->allowsVehicleClass(SVC_PEDESTRIAN) && !l->allowsVehicleClass(SVC_PASSENGER);
}

const MSEdge* opposite(const MSEdge* e) {
    const MSEdge* found = nullptr;
    for (const MSEdge* o : e->getToJunction()->getOutgoing()) {
        if (o->isNormal() && o->getToJunction() == e->getFromJunction() && o != e) {
            if (o->getStreetName() == e->getStreetName()) {
                return o;
            }
            if (found == nullptr) {
                found = o;
            }
        }
    }
    return found;
}

} // namespace


// ===========================================================================
// method definitions
// ===========================================================================
void
MSProntuario::insertOptions(OptionsCont& oc) {
    oc.doRegister("prontuario", new Option_Bool(true));
    oc.addDescription("prontuario", "Output", TL("Collect street indicators of the 'Prontuario de urbanismo y movilidad' in sumo-gui (right click on a street)"));
    oc.doRegister("prontuario-output", new Option_FileName());
    oc.addDescription("prontuario-output", "Output", TL("Write the prontuario street indicators at the end of the simulation into FILE (.txt: one report per street, otherwise a table)"));
}


void
MSProntuario::install(const OptionsCont& oc, bool gui) {
    myNet = nullptr;
    myMotor = myCars = myBikes = myBuses = myPeds = myEmissions = myNoise = nullptr;
    myCrossings.clear();
    const bool wanted = (gui && oc.exists("prontuario") && oc.getBool("prontuario")) || (oc.exists("prontuario-output") && oc.isSet("prontuario-output"));
    if (!wanted) {
        return;
    }
    myNet = MSNet::getInstance();
    myBegin = string2time(oc.getString("begin"));
    MSDetectorControl& dc = myNet->getDetectorControl();
    myMotor = new ProntuarioNet("prontuario_motor", myBegin, SVC_ROAD_MOTOR_CLASSES, 0);
    myCars = new ProntuarioNet("prontuario_coches", myBegin, SVC_PASSENGER | SVC_HOV | SVC_TAXI | SVC_PRIVATE, 0);
    myBikes = new ProntuarioNet("prontuario_bicis", myBegin, SVC_BICYCLE, 0);
    myBuses = new ProntuarioNet("prontuario_buses", myBegin, SVC_BUS | SVC_COACH, 0);
    myPeds = new ProntuarioNet("prontuario_peatones", myBegin, SVCAll, (int)PersonMode::WALK);
    myEmissions = new MSMeanData_Emissions("prontuario_emisiones", myBegin, SUMOTime_MAX, false, "false", false, false,
                                           100000, 0, "", "", std::vector<MSEdge*>(), AggregateType::NO);
    myNoise = new MSMeanData_Harmonoise("prontuario_ruido", myBegin, SUMOTime_MAX, false, "false", false, false,
                                        100000, 0, "", "", std::vector<MSEdge*>(), AggregateType::NO);
    for (MSMeanData* md : {
                myMotor, myCars, myBikes, myBuses, myPeds, myEmissions, myNoise
            }) {
        dc.addInMemory(md);
    }
    buildCrossingMap();
}


void
MSProntuario::buildCrossingMap() {
    myCrossings.clear();
    for (const MSEdge* e : MSEdge::getAllEdges()) {
        if (e->isCrossing()) {
            const std::string& id = e->getID();
            const std::string::size_type k = id.rfind("_c");
            if (id.size() > 1 && id[0] == ':' && k != std::string::npos && k > 1) {
                myCrossings[id.substr(1, k - 1)].push_back(e);
            }
        }
    }
}


bool
MSProntuario::active() {
    return myNet != nullptr && MSNet::hasInstance() && MSNet::getInstance() == myNet && myMotor != nullptr;
}


double
MSProntuario::elapsed() {
    if (!active()) {
        return 0;
    }
    return STEPS2TIME(myNet->getCurrentTimeStep() - myBegin);
}


void
MSProntuario::compute(const MSEdge* e, Key& k, std::vector<std::string>* out) {
    const double T = elapsed();
    const double hours = T / 3600.;
    std::vector<const MSLane*> car, bus;
    int sidewalks = 0;
    for (const MSLane* l : e->getLanes()) {
        if (isCarLane(l)) {
            car.push_back(l);
        } else if (isBusOnly(l)) {
            bus.push_back(l);
        } else if (isSidewalk(l)) {
            sidewalks++;
        }
    }
    const int nCar = (int)car.size();
    const bool motorEdge = nCar > 0 || !bus.empty();
    double vfree = 0;
    for (const MSLane* l : (car.empty() ? std::vector<const MSLane*>(e->getLanes().begin(), e->getLanes().end()) : car)) {
        vfree = MAX2(vfree, l->getSpeedLimit());
    }
    const int vk = (int)std::round(vfree * 3.6);
    const MSEdge* op = opposite(e);
    auto count = [&](const MSMeanData * md, const MSEdge * edge) {
        const double a = value(md, edge, SUMO_ATTR_ENTERED);
        const double b = value(md, edge, SUMO_ATTR_DEPARTED);
        return has(a) ? a + (has(b) ? b : 0) : NO_DATA;
    };
    auto perHour = [&](double n) {
        return has(n) && hours > 0 ? n / hours : NO_DATA;
    };

    if (out != nullptr) {
        out->push_back("INDICADORES DEL PRONTUARIO DE URBANISMO Y MOVILIDAD");
        out->push_back("");
        out->push_back("Calle: " + (e->getStreetName().empty() ? std::string("(sin nombre)") : e->getStreetName()) + "    tramo " + e->getID());
        out->push_back(prettyType(e->getEdgeType()) + " | " + num(e->getLength()) + " m | " + toString(nCar) + (nCar == 1 ? " carril" : " carriles")
                       + (bus.empty() ? "" : " + carril bus") + " | límite " + toString(vk) + " km/h | sentido " + compass(e)
                       + (op != nullptr ? " | tiene sentido contrario" : ""));
        out->push_back("Datos: toda la simulación hasta ahora, " + hhmm(STEPS2TIME(myBegin)) + "-" + hhmm(STEPS2TIME(myBegin) + T)
                       + " (" + num(T / 60.) + " min de simulación).");
        out->push_back("Valoración según el prontuario: ÓPTIMO / ADECUADO / INSUFICIENTE. * = gradación orientativa.");
    }
    if (T <= 0) {
        if (out != nullptr) {
            out->push_back("");
            out->push_back("Todavía no hay datos: inicia la simulación y vuelve a abrir (o actualiza) esta ventana.");
        }
        return;
    }

    // ---------------- traffic ----------------
    const double nMotor = count(myMotor, e);
    const double motorH = perHour(nMotor);
    const double speedCars = value(myCars, e, SUMO_ATTR_SPEED);
    const double speedMotor = value(myMotor, e, SUMO_ATTR_SPEED);
    const double speed = has(speedCars) && speedCars > 0 ? speedCars : speedMotor;
    const MSMeanData* speedSrc = has(speedCars) && speedCars > 0 ? myCars : myMotor;
    const double nSpeed = count(speedSrc, e);
    section(out, "TRÁFICO Y CONGESTIÓN (vehículos a motor)");
    // capacity
    double cap = 0;
    bool actuated = false;
    std::set<std::string> capNotes;
    for (const MSLane* l : car) {
        double gc = -1, cyc = 0;
        for (const MSLink* link : l->getLinkCont()) {
            const MSTrafficLightLogic* tl = link->getTLLogic();
            if (tl != nullptr && link->getTLIndex() >= 0) {
                const TLStat st = tlStats(tl, link->getTLIndex());
                actuated |= st.actuated;
                if (st.cycle > 0 && st.green / st.cycle > gc) {
                    gc = st.green / st.cycle;
                    cyc = st.cycle;
                }
            }
        }
        if (gc >= 0) {
            cap += 1800 * gc;
            capNotes.insert("semáforo g/C " + num(gc, 2) + " (ciclo " + num(cyc) + " s) x 1.800");
        } else {
            const bool high = vfree >= 22.2 || e->getEdgeType().find("motorway") != std::string::npos || e->getEdgeType().find("trunk") != std::string::npos;
            const double c = high ? 2000 : (nCar >= 2 ? 900 : 650);
            cap += c;
            capNotes.insert(high ? "vía de alta capacidad 2.000" : (nCar >= 2 ? "avenida sin semáforo 900 (rango 700-1.100)" : "colectora o local 650 (rango 500-800)"));
        }
    }
    if (nCar > 0 && has(motorH)) {
        k.flow = motorH;
        row(out, "Intensidad", num(motorH) + " veh/h", DATO, num(motorH / nCar) + " veh/h por carril; " + num(nMotor) + " vehículos en " + num(T / 60.) + " min");
        const double pl = motorH / nCar;
        const std::string ty = pl < 150 ? "calle local de barrio" : pl < 300 ? "entre local y colectora" : pl <= 600 ? "colectora o distribuidora" : pl <= 1000 ? "avenida urbana" : pl < 1500 ? "avenida muy cargada" : "vía de alta capacidad";
        row(out, "Tipo de vía según su intensidad", ty, DATO, "punta por carril: local < 150; colectora 300-600; avenida 600-1.000; alta capacidad 1.500-2.000 veh/h");
        if (cap > 0) {
            const double ic = motorH / cap;
            k.ic = ic;
            std::string capTxt;
            for (const std::string& s : capNotes) {
                capTxt += (capTxt.empty() ? "" : "; ") + s;
            }
            row(out, "Relación intensidad / capacidad (I/C) *", num(ic, 2),
                ic > 0.9 || ic < 0.3 ? INSUF : (ic >= 0.75 || ic < 0.4 ? ADEC : OPT),
                "Óptimo 0,4-0,75 (fluida); Adecuado 0,75-0,9 (densa); Insuficiente > 0,9 (congestionada) o < 0,3-0,4 (sobra calzada: candidata a dieta viaria)",
                "Capacidad estimada " + num(cap) + " veh/h: " + capTxt + "." + (actuated ? " Semáforo actuado: duraciones del programa." : ""), "práctica técnica, HCM");
        }
    } else if (nCar > 0) {
        row(out, "Intensidad e I/C", "-", ND, "", "Ningún vehículo a motor ha pasado todavía por el tramo.");
    }
    if (nCar > 0 && has(speed) && speed > 0 && vfree > 0) {
        const double rel = speed / vfree;
        const std::string los = rel > .85 ? "A" : rel > .67 ? "B" : rel > .5 ? "C" : rel > .4 ? "D" : rel > .3 ? "E" : "F";
        k.rel = rel;
        k.los = los;
        row(out, "Velocidad de recorrido / velocidad libre", num(rel * 100) + " %  (nivel " + los + ")", rel > .67 ? OPT : rel > .4 ? ADEC : INSUF,
            "Óptimo > 67 % (A-B; A > 85 %); Adecuado 40-67 % (C-D); Insuficiente <= 40 % (E-F; F <= 30 %)",
            "Velocidad media " + num(speed * 3.6, 1) + " km/h frente a " + toString(vk) + " km/h de velocidad libre (límite).", "HCM");
        const double ci = MAX2(0., (vfree / speed - 1) * 100);
        k.congestion = ci;
        row(out, "Índice de congestión *", ci > 999 ? "> 999 %" : num(ci) + " %", ci < 15 ? OPT : ci <= 35 ? ADEC : INSUF,
            "Óptimo < 15 % (baja); Adecuado 15-35 % (moderada); Insuficiente > 35 % (alta). (tiempo real - tiempo libre) / tiempo libre",
            "Recorrer el tramo cuesta " + num(e->getLength() / speed, 1) + " s; con la vía libre, " + num(e->getLength() / vfree, 1) + " s."
            + (has(nSpeed) && nSpeed < 5 ? " Muestra pequeña (" + num(nSpeed) + " veh): tómalo con cautela." : ""), "TomTom (umbrales didácticos)");
        const double tl = value(speedSrc, e, SUMO_ATTR_TIMELOSS);
        const double wt = value(speedSrc, e, SUMO_ATTR_WAITINGTIME);
        if (has(nSpeed) && nSpeed > 0 && has(tl)) {
            row(out, "Demora y tiempo parado por vehículo", num(tl / nSpeed, 1) + " s / " + num(has(wt) ? wt / nSpeed : NO_DATA, 1) + " s", DATO);
        }
        const double dens = value(myMotor, e, SUMO_ATTR_LANEDENSITY);
        if (has(dens)) {
            row(out, "Densidad media", num(dens, 1) + " veh/km y carril", DATO);
        }
        if (has(k.ic) && k.ic < 0.6 && ci > 35) {
            row(out, "Diagnóstico", "demora sin falta de capacidad", DATO, "",
                "El tramo va lento aunque su I/C es bajo: la causa suele estar fuera de él (cola desde el cruce siguiente, un giro difícil o vehículos bloqueados). Mira los tramos de salida.");
        }
    }
    if (nCar > 0 && has(motorH)) {
        const double opH = op != nullptr ? perHour(count(myMotor, op)) : NO_DATA;
        const double imd = (motorH + (has(opH) ? opH : 0)) / 0.09;
        const std::string ty = imd < 3000 ? "calle local (< 3.000)" : imd < 5000 ? "entre local y colectora" : imd <= 15000 ? "colectora (5.000-15.000)" : imd <= 40000 ? "avenida 2+2 (15.000-40.000)" : imd < 80000 ? "vía principal" : "alta capacidad (>= 80.000)";
        row(out, "IMD estimada (dos sentidos)", num(imd) + " veh/día", DATO, "IMD = intensidad horaria / K (0,09). Tipo de vía: " + ty,
            std::string("Solo es una estimación si lo simulado es la hora punta.") + (op == nullptr ? " Sin sentido contrario: un solo sentido." : ""));
        int opCar = 0;
        if (op != nullptr) {
            for (const MSLane* l : op->getLanes()) {
                opCar += isCarLane(l) ? 1 : 0;
            }
        }
        if (nCar == 2 && opCar == 2) {
            row(out, "Dieta viaria (4 -> 3 carriles) *", num(imd) + " veh/día", imd <= 20000 ? OPT : imd <= 25000 ? ADEC : INSUF,
                "Óptimo <= 20.000 (buen candidato); Adecuado 20.000-25.000; Insuficiente > 25.000 (requiere estudio detallado)", "", "FHWA");
        } else {
            row(out, "Dieta viaria (4 -> 3 carriles) *", "no aplica", NA, "", "Solo para calles de 2 + 2 carriles; esta tiene " + toString(nCar) + (op != nullptr ? " + " + toString(opCar) : "") + ".");
        }
    }

    // ---------------- design ----------------
    if (nCar > 0) {
        section(out, "DISEÑO DE LA CALZADA");
        const bool urban = e->getEdgeType().find("motorway") == std::string::npos && e->getEdgeType().find("trunk") == std::string::npos;
        Grade g;
        std::string tx;
        if (!urban) {
            g = NA;
            tx = "Vía interurbana: límite según RGC art. 48.";
        } else if (vk > 50) {
            g = INSUF;
            tx = "Más de 50 km/h en zona urbana.";
        } else if (vk <= 30) {
            g = OPT;
            tx = vk <= 20 ? "20 km/h: plataforma única." : "30 km/h generalizado.";
        } else if (nCar >= 2) {
            g = ADEC;
            tx = "50 km/h con 2 o más carriles por sentido (RGC art. 50).";
        } else {
            g = INSUF;
            tx = "Con un carril por sentido el límite genérico urbano es 30 km/h (RGC art. 50).";
        }
        row(out, "Velocidad en vías urbanas *", toString(vk) + " km/h", g,
            "Óptimo 30 km/h generalizado (20 en plataforma única); Adecuado 30 (1 carril por sentido) o 50 (>= 2 carriles); Insuficiente > 50", tx, "RGC, OMS");
        double w = 0;
        for (const MSLane* l : car) {
            w += l->getWidth();
        }
        w /= nCar;
        Grade gw;
        std::string um;
        if (vk <= 30) {
            gw = w >= 3.5 ? INSUF : w <= 3.0 ? OPT : ADEC;
            um = "A 30 km/h: Óptimo 2,75-3,00 m; Insuficiente >= 3,50 m (invita a correr)";
        } else if (vk <= 50) {
            gw = w >= 3.5 ? INSUF : ADEC;
            um = "A 50 km/h: Adecuado 3,00-3,25 m; Insuficiente >= 3,50 m";
        } else {
            gw = NA;
            um = "Fuera de ciudad lo fija la Norma 3.1-IC (3,50 m)";
        }
        row(out, "Ancho de carril urbano *", num(w, 2) + " m", gw, um, "", "IVP, CALMAR");
        const double d = vfree * 1.0 + vfree * vfree / (2 * 0.7 * 9.81);
        row(out, "Distancia de detención al límite", num(d, 1) + " m", DATO, "d = v t + v² / (2 μ g), t = 1 s, μ = 0,7. Referencia: 13,4 m a 30 km/h; 27,9 m a 50 km/h");
        row(out, "Velocidad real (V85) respecto al límite *", "no calculable", NA, "", "La V85 necesita velocidades individuales: usa un detector E1 o la salida FCD.");
        if (!bus.empty()) {
            row(out, "Carril bus", toString((int)bus.size()), DATO, "capacidad práctica 60-90 autobuses/h");
        }
    }

    // ---------------- pedestrians ----------------
    const double pedsH = perHour(count(myPeds, e));
    const double pedsOp = op != nullptr ? perHour(count(myPeds, op)) : NO_DATA;
    if (sidewalks > 0 || motorEdge) {
        section(out, "PEATONES, PASOS Y SEMÁFOROS");
        if (has(pedsH)) {
            k.peds = pedsH;
            row(out, "Peatones por las aceras del tramo", num(pedsH) + " peatones/h", DATO, op != nullptr ? "sentido contrario: " + num(pedsOp) + " peatones/h" : "");
        }
        if (myCrossings.empty()) {
            row(out, "Pasos de peatones", "-", ND, "", "La red no tiene pasos de peatones modelados (netconvert --crossings.guess o netedit).");
        } else if (motorEdge) {
            std::set<std::string> ids = {e->getID()};
            if (op != nullptr) {
                ids.insert(op->getID());
            }
            double vehBoth = (has(motorH) ? motorH : 0);
            int lanesBoth = 0;
            for (const MSLane* l : e->getLanes()) {
                lanesBoth += isSidewalk(l) ? 0 : 1;
            }
            if (op != nullptr) {
                const double o = perHour(count(myMotor, op));
                vehBoth += has(o) ? o : 0;
                for (const MSLane* l : op->getLanes()) {
                    lanesBoth += isSidewalk(l) ? 0 : 1;
                }
            }
            const double pedBoth = (has(pedsH) ? pedsH : 0) + (has(pedsOp) ? pedsOp : 0);
            int found = 0;
            const std::pair<std::string, const MSJunction*> ends[2] = {{"inicio", e->getFromJunction()}, {"final", e->getToJunction()}};
            for (const auto& end : ends) {
                auto it = myCrossings.find(end.second->getID());
                if (it == myCrossings.end()) {
                    continue;
                }
                for (const MSEdge* c : it->second) {
                    bool crossesStreet = false;
                    for (const std::string& x : c->getCrossingEdges()) {
                        crossesStreet |= ids.count(x) > 0;
                    }
                    if (!crossesStreet) {
                        continue;
                    }
                    found++;
                    const MSLane* cl = c->getLanes().front();
                    const double L = cl->getLength();
                    if (out != nullptr) {
                        out->push_back("  * Paso de peatones al " + end.first + " del tramo (" + c->getID() + ")");
                    }
                    row(out, "  Refugio central en cruces largos", num(L, 1) + " m de cruce", L > 14 ? INSUF : L > 12 ? ADEC : OPT,
                        "Óptimo: no hace falta (<= 12 m); Adecuado: refugio >= 1,2 m en cruces de 12-14 m; Insuficiente: > 14 m sin refugio", "", "IVP");
                    row(out, "  Ancho del paso *", num(cl->getWidth(), 1) + " m", cl->getWidth() < 3.95 ? INSUF : cl->getWidth() > 4.05 ? OPT : ADEC,
                        "Adecuado 4 m (IVP Madrid); Óptimo > 4 m con mucha afluencia; Insuficiente < 4 m", "", "IVP");
                    const MSLink* tlLink = nullptr;
                    for (const MSLane::IncomingLaneInfo& inc : cl->getIncomingLanes()) {
                        if (inc.viaLink != nullptr && inc.viaLink->getTLLogic() != nullptr && inc.viaLink->getTLIndex() >= 0) {
                            tlLink = inc.viaLink;
                            break;
                        }
                    }
                    if (tlLink != nullptr) {
                        const TLStat st = tlStats(tlLink->getTLLogic(), tlLink->getTLIndex());
                        const double req = L / 0.5;
                        row(out, "  Tiempo de verde peatonal", num(st.maxGreen) + " s", st.maxGreen < req ? INSUF : st.maxGreen < req + 6 ? ADEC : OPT,
                            "Necesario " + num(req) + " s (longitud / 0,5 m/s, Orden TMA/851/2021); Óptimo " + num(req + 6) + " s (Comunidad de Madrid: + 3 s + 3 s)",
                            st.actuated ? "Semáforo actuado: verde del programa." : "", "TMA, CAM");
                        row(out, "  Rojo peatonal (espera) *", num(st.maxRed) + " s", st.maxRed > 60 ? INSUF : st.maxRed > 40 ? ADEC : OPT,
                            "Óptimo <= 40 s; Adecuado <= 60 s; Insuficiente > 60 s (incita a cruzar en rojo). Ciclo " + num(st.cycle) + " s", "", "CALMAR");
                        const bool just = vehBoth > 1000 || pedBoth > 150;
                        row(out, "  Paso cebra o semaforizado", "semaforizado", ADEC, "Semaforizar si > 150 peatones/h durante 8 h o > 1.000 veh/h",
                            just ? "El semáforo está justificado por la intensidad." : "Con " + num(vehBoth) + " veh/h y " + num(pedBoth) + " peatones/h podría bastar un paso de cebra en una calle templada.", "IVP");
                    } else {
                        const bool bad = vehBoth > 1000 || pedBoth > 450 || lanesBoth > 4;
                        row(out, "  Paso cebra o semaforizado", "paso de cebra", bad ? INSUF : OPT,
                            "Óptimo: cebra en calles templadas con tráfico y peatones bajos o medios; Insuficiente: cebra con > 1.000-1.200 veh/h, > 400-500 peatones/h o más de 4 carriles",
                            "Calle con " + num(vehBoth) + " veh/h, " + num(pedBoth) + " peatones/h y " + toString(lanesBoth) + " carriles en total." + (bad ? " Conviene semaforizar." : ""), "IVP");
                    }
                }
            }
            // distance between consecutive crossings along the street
            const std::string name = e->getStreetName();
            auto hasCrossing = [&](const MSJunction * j) {
                auto it = myCrossings.find(j->getID());
                if (it == myCrossings.end()) {
                    return false;
                }
                std::set<std::string> street = ids;
                if (!name.empty()) {
                    for (const MSEdge* x : j->getIncoming()) {
                        if (x->getStreetName() == name) {
                            street.insert(x->getID());
                        }
                    }
                    for (const MSEdge* x : j->getOutgoing()) {
                        if (x->getStreetName() == name) {
                            street.insert(x->getID());
                        }
                    }
                }
                for (const MSEdge* c : it->second) {
                    for (const std::string& x : c->getCrossingEdges()) {
                        if (street.count(x) > 0) {
                            return true;
                        }
                    }
                }
                return false;
            };
            auto walk = [&](bool fwd, double & dist) {
                const MSEdge* cur = e;
                const MSJunction* j = fwd ? e->getToJunction() : e->getFromJunction();
                for (int guard = 0; guard < 80; guard++) {
                    if (hasCrossing(j)) {
                        return true;
                    }
                    if (name.empty()) {
                        return false;
                    }
                    const MSEdge* next = nullptr;
                    for (const MSEdge* x : (fwd ? j->getOutgoing() : j->getIncoming())) {
                        if (x->isNormal() && x != cur && x->getStreetName() == name
                                && (fwd ? x->getToJunction() != cur->getFromJunction() : x->getFromJunction() != cur->getToJunction())) {
                            bool motor = false;
                            for (const MSLane* l : x->getLanes()) {
                                motor |= isCarLane(l) || isBusOnly(l);
                            }
                            if (motor) {
                                next = x;
                                break;
                            }
                        }
                    }
                    if (next == nullptr) {
                        return false;
                    }
                    dist += next->getLength();
                    cur = next;
                    j = fwd ? next->getToJunction() : next->getFromJunction();
                    if (dist > 3000) {
                        return false;
                    }
                }
                return false;
            };
            double df = 0, db = 0;
            const bool cf = walk(true, df), cb = walk(false, db);
            const double tot = e->getLength() + df + db;
            if (cf && cb) {
                row(out, "Distancia entre pasos de peatones *", num(tot) + " m", tot > 100 ? INSUF : tot > 80 ? ADEC : OPT,
                    "Óptimo 50-80 m (<= 75 m en áreas centrales); Adecuado <= 100 m; Insuficiente > 100 m",
                    "Distancia por la calle entre los dos pasos que encierran este tramo.", "CALMAR, IVP");
            } else {
                row(out, "Distancia entre pasos de peatones *", "> " + num(tot) + " m", tot > 100 ? INSUF : ND, "Adecuado <= 100 m",
                    std::string("No hay paso de peatones en ") + (cf || cb ? "uno de los extremos" : "ningún extremo") + " a lo largo de la calle.", "CALMAR, IVP");
            }
            if (found == 0) {
                row(out, "Pasos en los extremos del tramo", "ninguno", DATO, "", "No hay pasos de peatones sobre esta calle en sus dos cruces.");
            }
        }
    }

    // ---------------- public transport ----------------
    const double nBus = count(myBuses, e);
    const double busH = perHour(nBus);
    if (motorEdge) {
        section(out, "TRANSPORTE PÚBLICO");
        if (has(busH) && busH > 0) {
            const double headway = 60. / busH;
            k.busHeadway = headway;
            row(out, "Intervalo de paso del autobús *", num(headway, 1) + " min", headway < 10 ? OPT : headway <= 20 ? ADEC : INSUF,
                num(busH, 1) + " autobuses/h. Óptimo < 10 min (A); Adecuado 10-20 (B-C); Insuficiente > 20 (D-F)",
                "Todas las líneas que pasan por el tramo juntas. Espera media " + num(headway / 2, 1) + " min.", "TCQSM");
            row(out, "Oferta de plazas", num(busH * 90) + " plazas/h", DATO, "autobuses/h x 90 plazas (autobús estándar de 12 m: 70-95)");
            const double vb = value(myBuses, e, SUMO_ATTR_SPEED);
            if (has(vb) && vb > 0) {
                row(out, "Velocidad del autobús en el tramo *", num(vb * 3.6, 1) + " km/h", vb * 3.6 > 15 ? OPT : vb * 3.6 >= 12 ? ADEC : INSUF,
                    "Óptimo > 15 km/h; Adecuado 12-15; Insuficiente < 10-12 (centro congestionado)",
                    "Incluye paradas y semáforos de este tramo; la velocidad comercial del prontuario es la de toda la línea.", "práctica técnica");
                if (has(speed) && speed > 0) {
                    const double comp = speed / vb;
                    row(out, "Competitividad en el tramo *", num(comp, 2), comp < 1.4 ? OPT : comp <= 2.2 ? ADEC : INSUF,
                        "tiempo en bus / tiempo en coche. Óptimo < 1,4; Adecuado 1,4-2,2; Insuficiente > 2,2",
                        "El prontuario lo mide puerta a puerta; aquí solo se compara este tramo.", "IDAE");
                }
            }
            if (!bus.empty()) {
                row(out, "Carga del carril bus", num(busH, 1) + " autobuses/h", busH > 90 ? INSUF : busH >= 60 ? ADEC : OPT, "Capacidad práctica 60-90 autobuses/h");
            }
        } else {
            row(out, "Autobuses", "ninguno", DATO, "", "Ningún autobús ha recorrido este tramo.");
        }
    }

    // ---------------- people ----------------
    const double carsH = perHour(count(myCars, e));
    const double bikesH = perHour(count(myBikes, e));
    if (motorEdge && out != nullptr) {
        const double pc = has(carsH) ? carsH * 1.25 : 0;
        const double pb = has(busH) ? busH * 30 : 0;
        const double pk = has(bikesH) ? bikesH : 0;
        const double pp = has(pedsH) ? pedsH : 0;
        const double tot = pc + pb + pk + pp;
        if (tot > 0) {
            section(out, "PERSONAS QUE PASAN POR LA SECCIÓN");
            row(out, "Reparto en esta sección", num(tot) + " personas/h", DATO,
                "coche " + num(pc / tot * 100) + " %; autobús " + num(pb / tot * 100) + " %; bici " + num(pk / tot * 100) + " %; a pie " + num(pp / tot * 100) + " %",
                "Supuestos: 1,25 personas por coche y 30 viajeros por autobús. El umbral del prontuario (coche < 25 %, óptimo < 10 %) se refiere a los viajes internos de toda la ciudad.", "OMM, AEUB");
        }
    }

    // ---------------- environment ----------------
    section(out, "RUIDO Y EMISIONES");
    const double noise = value(myNoise, e, SUMO_ATTR_NOISE);
    if (has(noise) && noise > 0) {
        k.noise = noise;
        row(out, "Ruido del tráfico", num(noise, 1) + " dB(A)", noise > 65 ? INSUF : noise > 53 ? ADEC : OPT,
            "Óptimo <= 53 dB (OMS, Lden); Adecuado <= 65 dB (RD 1367/2007, Ld residencial); Insuficiente > 65 dB",
            "Harmonoise da el ruido emitido junto a la vía, no el Lden en fachada: sirve para comparar calles y escenarios.", "RD1367, OMS");
    } else {
        row(out, "Ruido del tráfico", "-", ND, "", "Sin vehículos en el tramo.");
    }
    const double co2 = value(myEmissions, e, SUMO_ATTR_CO2_ABS);
    if (has(co2) && co2 > 0) {
        const double kgh = co2 / 1e6 / hours;
        k.co2 = kgh;
        const double dist = value(myMotor, e, SUMO_ATTR_DISTANCE);
        const double gkm = has(dist) && dist > 0 ? (co2 / 1000.) / (dist / 1000.) : NO_DATA;
        row(out, "CO2 emitido", num(kgh, 2) + " kg/h", DATO, has(gkm) ? num(gkm) + " g por vehículo y km; unos " + num(gkm / 1.25) + " g por viajero y km (1,25 personas por coche)" : "");
        const double nox = value(myEmissions, e, SUMO_ATTR_NOX_ABS);
        const double pmx = value(myEmissions, e, SUMO_ATTR_PMX_ABS);
        row(out, "NOx y partículas (PMx)", num(has(nox) ? nox / 1000 / hours : NO_DATA, 1) + " g/h / " + num(has(pmx) ? pmx / 1000 / hours : NO_DATA, 2) + " g/h", DATO, "",
            "El prontuario valora el aire con concentraciones (µg/m³) medidas en estaciones; SUMO calcula emisiones en gramos. Sirven para comparar, no para comprobar límites legales.");
    }
}


std::string
MSProntuario::report(const MSEdge* edge) {
    if (!active()) {
        return "Los indicadores del prontuario no están activos (opción --prontuario).";
    }
    if (edge == nullptr || !edge->isNormal()) {
        return "Elige una calle normal (no un cruce interno).";
    }
    Key k;
    std::vector<std::string> lines;
    compute(edge, k, &lines);
    std::string result;
    for (const std::string& l : lines) {
        result += l + "\n";
    }
    return result;
}


void
MSProntuario::writeOutput() {
    if (!active() || !OptionsCont::getOptions().isSet("prontuario-output")) {
        return;
    }
    const std::string file = OptionsCont::getOptions().getString("prontuario-output");
    OutputDevice& dev = OutputDevice::getDevice(file);
    const bool reports = StringUtils::endsWith(StringUtils::to_lower_case(file), ".txt");
    if (!reports) {
        dev << "tramo;calle;tipo;sentido;longitud_m;carriles;limite_kmh;veh_h;IC;vel_relativa_pct;nivel_servicio;congestion_pct;peatones_h;intervalo_bus_min;ruido_dBA;CO2_kg_h\n";
    }
    for (const MSEdge* e : MSEdge::getAllEdges()) {
        if (!e->isNormal()) {
            continue;
        }
        bool motor = false;
        for (const MSLane* l : e->getLanes()) {
            motor |= isCarLane(l) || isBusOnly(l);
        }
        if (!motor) {
            continue;
        }
        if (reports) {
            dev << report(e) << "\n==========================================================================================\n\n";
        } else {
            Key k;
            compute(e, k, nullptr);
            int nCar = 0;
            double vfree = 0;
            for (const MSLane* l : e->getLanes()) {
                if (isCarLane(l)) {
                    nCar++;
                    vfree = MAX2(vfree, l->getSpeedLimit());
                }
            }
            auto f = [](double v, int d) {
                return has(v) ? num(v, d) : std::string("");
            };
            std::string name = e->getStreetName();
            std::replace(name.begin(), name.end(), ';', ',');
            dev << e->getID() << ";" << name << ";" << prettyType(e->getEdgeType()) << ";" << compass(e) << ";"
                << num(e->getLength()) << ";" << nCar << ";" << (int)std::round(vfree * 3.6) << ";"
                << f(k.flow, 0) << ";" << f(k.ic, 2) << ";" << f(has(k.rel) ? k.rel * 100 : k.rel, 0) << ";" << k.los << ";"
                << f(k.congestion, 0) << ";" << f(k.peds, 0) << ";" << f(k.busHeadway, 1) << ";"
                << f(k.noise, 1) << ";" << f(k.co2, 2) << "\n";
        }
    }
    dev.close();
}


/****************************************************************************/
