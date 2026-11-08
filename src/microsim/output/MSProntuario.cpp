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
#include <fstream>
#include <regex>
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
#include <utils/iodevices/XLSXWriter.h>
#include <utils/common/FileHelpers.h>
#include <utils/xml/XMLSubSys.h>
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
std::map<const MSEdge*, double> MSProntuario::myCongestionCache;
std::map<std::string, std::pair<double, double> > MSProntuario::myFieldCounts;
SUMOTime MSProntuario::myCongestionCacheTime = -1;
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
            return "[\303\223PTIMO]";
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

/// @brief collector for table output
std::vector<MSProntuario::Row>* gRows = nullptr;
std::string gSection;

/// @brief one graded indicator
void row(std::vector<std::string>* out, const std::string& name, const std::string& val, Grade g,
         const std::string& umbral = "", const std::string& note = "", const std::string& src = "") {
    if (gRows != nullptr) {
        MSProntuario::Row r;
        r.section = gSection;
        std::string n = name;
        while (!n.empty() && n[0] == ' ') {
            n = n.substr(1);
        }
        r.name = n;
        r.value = val.empty() ? "-" : val;
        std::string gt = gradeText(g);
        r.grade = gt.substr(1, gt.size() - 2);
        r.gradeCode = (int)g;
        r.umbral = umbral;
        r.note = note;
        r.source = src;
        gRows->push_back(r);
    }
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
    gSection = title;
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
        {"highway.motorway", "autopista"}, {"highway.trunk", "autov\303\255a"}, {"highway.primary", "v\303\255a primaria"},
        {"highway.secondary", "v\303\255a secundaria"}, {"highway.tertiary", "v\303\255a terciaria"},
        {"highway.residential", "calle residencial"}, {"highway.living_street", "calle de convivencia"},
        {"highway.service", "v\303\255a de servicio"}, {"highway.unclassified", "v\303\255a sin clasificar"},
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
    return t.empty() ? "v\303\255a" : t;
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
    myCongestionCache.clear();
    myCongestionCacheTime = -1;
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
    loadFieldCounts();
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
MSProntuario::congestionIndex(const MSEdge* edge) {
    if (!active() || edge == nullptr || !edge->isNormal()) {
        return -1;
    }
    const SUMOTime now = myNet->getCurrentTimeStep();
    if (now != myCongestionCacheTime) {
        myCongestionCache.clear();
        myCongestionCacheTime = now;
    }
    auto it = myCongestionCache.find(edge);
    if (it != myCongestionCache.end()) {
        return it->second;
    }
    double result = -1;
    double vfree = 0;
    for (const MSLane* l : edge->getLanes()) {
        if (isCarLane(l)) {
            vfree = MAX2(vfree, l->getSpeedLimit());
        }
    }
    double speed = value(myCars, edge, SUMO_ATTR_SPEED);
    if (!has(speed) || speed <= 0) {
        speed = value(myMotor, edge, SUMO_ATTR_SPEED);
    }
    if (vfree > 0 && has(speed) && speed > 0) {
        result = MAX2(0., (vfree / speed - 1) * 100);
    }
    myCongestionCache[edge] = result;
    return result;
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
                       + (bus.empty() ? "" : " + carril bus") + " | l\303\255mite " + toString(vk) + " km/h | sentido " + compass(e)
                       + (op != nullptr ? " | tiene sentido contrario" : ""));
        out->push_back("Datos: toda la simulaci\303\263n hasta ahora, " + hhmm(STEPS2TIME(myBegin)) + "-" + hhmm(STEPS2TIME(myBegin) + T)
                       + " (" + num(T / 60.) + " min de simulaci\303\263n).");
        out->push_back("Valoraci\303\263n seg\303\272n el prontuario: \303\223PTIMO / ADECUADO / INSUFICIENTE. * = gradaci\303\263n orientativa.");
    }
    if (T <= 0) {
        if (out != nullptr) {
            out->push_back("");
            out->push_back("Todav\303\255a no hay datos: inicia la simulaci\303\263n y vuelve a abrir (o actualiza) esta ventana.");
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
    section(out, "TR\303\201FICO Y CONGESTI\303\223N (veh\303\255culos a motor)");
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
            capNotes.insert("sem\303\241foro g/C " + num(gc, 2) + " (ciclo " + num(cyc) + " s) x 1.800");
        } else {
            const bool high = vfree >= 22.2 || e->getEdgeType().find("motorway") != std::string::npos || e->getEdgeType().find("trunk") != std::string::npos;
            const double c = high ? 2000 : (nCar >= 2 ? 900 : 650);
            cap += c;
            capNotes.insert(high ? "v\303\255a de alta capacidad 2.000" : (nCar >= 2 ? "avenida sin sem\303\241foro 900 (rango 700-1.100)" : "colectora o local 650 (rango 500-800)"));
        }
    }
    if (nCar > 0 && has(motorH)) {
        k.flow = motorH;
        row(out, "Intensidad", num(motorH) + " veh/h", DATO, num(motorH / nCar) + " veh/h por carril; " + num(nMotor) + " veh\303\255culos en " + num(T / 60.) + " min");
        const double pl = motorH / nCar;
        const std::string ty = pl < 150 ? "calle local de barrio" : pl < 300 ? "entre local y colectora" : pl <= 600 ? "colectora o distribuidora" : pl <= 1000 ? "avenida urbana" : pl < 1500 ? "avenida muy cargada" : "v\303\255a de alta capacidad";
        row(out, "Tipo de v\303\255a seg\303\272n su intensidad", ty, DATO, "punta por carril: local < 150; colectora 300-600; avenida 600-1.000; alta capacidad 1.500-2.000 veh/h");
        if (cap > 0) {
            const double ic = motorH / cap;
            k.ic = ic;
            std::string capTxt;
            for (const std::string& s : capNotes) {
                capTxt += (capTxt.empty() ? "" : "; ") + s;
            }
            row(out, "Relaci\303\263n intensidad / capacidad (I/C) *", num(ic, 2),
                ic > 0.9 || ic < 0.3 ? INSUF : (ic >= 0.75 || ic < 0.4 ? ADEC : OPT),
                "\303\223ptimo 0,4-0,75 (fluida); Adecuado 0,75-0,9 (densa); Insuficiente > 0,9 (congestionada) o < 0,3-0,4 (sobra calzada: candidata a dieta viaria)",
                "Capacidad estimada " + num(cap) + " veh/h: " + capTxt + "." + (actuated ? " Sem\303\241foro actuado: duraciones del programa." : ""), "pr\303\241ctica t\303\251cnica, HCM");
        }
    } else if (nCar > 0) {
        row(out, "Intensidad e I/C", "-", ND, "", "Ning\303\272n veh\303\255culo a motor ha pasado todav\303\255a por el tramo.");
    }
    if (nCar > 0 && has(speed) && speed > 0 && vfree > 0) {
        const double rel = speed / vfree;
        const std::string los = rel > .85 ? "A" : rel > .67 ? "B" : rel > .5 ? "C" : rel > .4 ? "D" : rel > .3 ? "E" : "F";
        k.rel = rel;
        k.los = los;
        row(out, "Velocidad de recorrido / velocidad libre", num(rel * 100) + " %  (nivel " + los + ")", rel > .67 ? OPT : rel > .4 ? ADEC : INSUF,
            "\303\223ptimo > 67 % (A-B; A > 85 %); Adecuado 40-67 % (C-D); Insuficiente <= 40 % (E-F; F <= 30 %)",
            "Velocidad media " + num(speed * 3.6, 1) + " km/h frente a " + toString(vk) + " km/h de velocidad libre (l\303\255mite).", "HCM");
        const double ci = MAX2(0., (vfree / speed - 1) * 100);
        k.congestion = ci;
        row(out, "\303\215ndice de congesti\303\263n *", ci > 999 ? "> 999 %" : num(ci) + " %", ci < 15 ? OPT : ci <= 35 ? ADEC : INSUF,
            "\303\223ptimo < 15 % (baja); Adecuado 15-35 % (moderada); Insuficiente > 35 % (alta). (tiempo real - tiempo libre) / tiempo libre",
            "Recorrer el tramo cuesta " + num(e->getLength() / speed, 1) + " s; con la v\303\255a libre, " + num(e->getLength() / vfree, 1) + " s."
            + (has(nSpeed) && nSpeed < 5 ? " Muestra peque\303\261a (" + num(nSpeed) + " veh): t\303\263malo con cautela." : ""), "TomTom (umbrales did\303\241cticos)");
        const double tl = value(speedSrc, e, SUMO_ATTR_TIMELOSS);
        const double wt = value(speedSrc, e, SUMO_ATTR_WAITINGTIME);
        if (has(nSpeed) && nSpeed > 0 && has(tl)) {
            row(out, "Demora y tiempo parado por veh\303\255culo", num(tl / nSpeed, 1) + " s / " + num(has(wt) ? wt / nSpeed : NO_DATA, 1) + " s", DATO);
        }
        const double dens = value(myMotor, e, SUMO_ATTR_LANEDENSITY);
        if (has(dens)) {
            row(out, "Densidad media", num(dens, 1) + " veh/km y carril", DATO);
        }
        if (has(k.ic) && k.ic < 0.6 && ci > 35) {
            row(out, "Diagn\303\263stico", "demora sin falta de capacidad", DATO, "",
                "El tramo va lento aunque su I/C es bajo: la causa suele estar fuera de \303\251l (cola desde el cruce siguiente, un giro dif\303\255cil o veh\303\255culos bloqueados). Mira los tramos de salida.");
        }
    }
    if (nCar > 0 && has(motorH)) {
        const double opH = op != nullptr ? perHour(count(myMotor, op)) : NO_DATA;
        const double imd = (motorH + (has(opH) ? opH : 0)) / 0.09;
        const std::string ty = imd < 3000 ? "calle local (< 3.000)" : imd < 5000 ? "entre local y colectora" : imd <= 15000 ? "colectora (5.000-15.000)" : imd <= 40000 ? "avenida 2+2 (15.000-40.000)" : imd < 80000 ? "v\303\255a principal" : "alta capacidad (>= 80.000)";
        row(out, "IMD estimada (dos sentidos)", num(imd) + " veh/d\303\255a", DATO, "IMD = intensidad horaria / K (0,09). Tipo de v\303\255a: " + ty,
            std::string("Solo es una estimaci\303\263n si lo simulado es la hora punta.") + (op == nullptr ? " Sin sentido contrario: un solo sentido." : ""));
        int opCar = 0;
        if (op != nullptr) {
            for (const MSLane* l : op->getLanes()) {
                opCar += isCarLane(l) ? 1 : 0;
            }
        }
        if (nCar == 2 && opCar == 2) {
            row(out, "Dieta viaria (4 -> 3 carriles) *", num(imd) + " veh/d\303\255a", imd <= 20000 ? OPT : imd <= 25000 ? ADEC : INSUF,
                "\303\223ptimo <= 20.000 (buen candidato); Adecuado 20.000-25.000; Insuficiente > 25.000 (requiere estudio detallado)", "", "FHWA");
        } else {
            row(out, "Dieta viaria (4 -> 3 carriles) *", "no aplica", NA, "", "Solo para calles de 2 + 2 carriles; esta tiene " + toString(nCar) + (op != nullptr ? " + " + toString(opCar) : "") + ".");
        }
    }

    // ---------------- field count and through traffic ----------------
    if (nCar > 0) {
        section(out, "AFORO DE CAMPO Y TR\303\201FICO DE PASO");
        const double fc = fieldCount(e);
        if (fc >= 0) {
            const double M = has(motorH) ? motorH : 0;
            const double geh = (M + fc) > 0 ? std::sqrt(2 * (M - fc) * (M - fc) / (M + fc)) : 0;
            k.geh = geh;
            row(out, "Aforo de campo", num(fc) + " veh/h", DATO,
                "simulado: " + num(M) + " veh/h" + (fc > 0 ? " (" + std::string(M >= fc ? "+" : "") + num((M - fc) / fc * 100) + " %)" : ""));
            row(out, "Ajuste al aforo (GEH) *", num(geh, 1), geh < 5 ? OPT : geh <= 10 ? ADEC : INSUF,
                "GEH = ra\303\255z(2 (M - C)\302\262 / (M + C)), M simulado y C aforo en veh/h. \303\223ptimo < 5 (el modelo reproduce la calle); Adecuado 5-10 (revisar); Insuficiente > 10",
                "Para acercar la simulaci\303\263n a los aforos: bot\303\263n derecho > \302\253Ajustar la demanda a los aforos de campo\302\273.", "DMRB, criterio de calibraci\303\263n");
        } else {
            row(out, "Aforo de campo", "no introducido", NA, "", "Bot\303\263n derecho sobre la calle > \302\253Introducir aforo de campo (veh/h)\302\273.");
        }
        double ff = -1;
        const std::string fringe = fringeInfo(e, ff);
        row(out, "Calle de borde (tr\303\241fico de paso)", fringe, DATO, ff > 0 ? "factor de tr\303\241fico de paso con que se gener\303\263 el escenario: " + num(ff, 1) : "",
            "El factor de osmWebWizard multiplica la probabilidad de que los viajes empiecen o acaben en las calles del borde de la zona. Solo act\303\272a al generar los viajes; para ajustar el tr\303\241fico de una calle usa su aforo de campo.");
    }

    // ---------------- design ----------------
    if (nCar > 0) {
        section(out, "DISE\303\221O DE LA CALZADA");
        const bool urban = e->getEdgeType().find("motorway") == std::string::npos && e->getEdgeType().find("trunk") == std::string::npos;
        Grade g;
        std::string tx;
        if (!urban) {
            g = NA;
            tx = "V\303\255a interurbana: l\303\255mite seg\303\272n RGC art. 48.";
        } else if (vk > 50) {
            g = INSUF;
            tx = "M\303\241s de 50 km/h en zona urbana.";
        } else if (vk <= 30) {
            g = OPT;
            tx = vk <= 20 ? "20 km/h: plataforma \303\272nica." : "30 km/h generalizado.";
        } else if (nCar >= 2) {
            g = ADEC;
            tx = "50 km/h con 2 o m\303\241s carriles por sentido (RGC art. 50).";
        } else {
            g = INSUF;
            tx = "Con un carril por sentido el l\303\255mite gen\303\251rico urbano es 30 km/h (RGC art. 50).";
        }
        row(out, "Velocidad en v\303\255as urbanas *", toString(vk) + " km/h", g,
            "\303\223ptimo 30 km/h generalizado (20 en plataforma \303\272nica); Adecuado 30 (1 carril por sentido) o 50 (>= 2 carriles); Insuficiente > 50", tx, "RGC, OMS");
        double w = 0;
        for (const MSLane* l : car) {
            w += l->getWidth();
        }
        w /= nCar;
        Grade gw;
        std::string um;
        if (vk <= 30) {
            gw = w >= 3.5 ? INSUF : w <= 3.0 ? OPT : ADEC;
            um = "A 30 km/h: \303\223ptimo 2,75-3,00 m; Insuficiente >= 3,50 m (invita a correr)";
        } else if (vk <= 50) {
            gw = w >= 3.5 ? INSUF : ADEC;
            um = "A 50 km/h: Adecuado 3,00-3,25 m; Insuficiente >= 3,50 m";
        } else {
            gw = NA;
            um = "Fuera de ciudad lo fija la Norma 3.1-IC (3,50 m)";
        }
        row(out, "Ancho de carril urbano *", num(w, 2) + " m", gw, um, "", "IVP, CALMAR");
        const double d = vfree * 1.0 + vfree * vfree / (2 * 0.7 * 9.81);
        row(out, "Distancia de detenci\303\263n al l\303\255mite", num(d, 1) + " m", DATO, "d = v t + v\302\262 / (2 \316\274 g), t = 1 s, \316\274 = 0,7. Referencia: 13,4 m a 30 km/h; 27,9 m a 50 km/h");
        row(out, "Velocidad real (V85) respecto al l\303\255mite *", "no calculable", NA, "", "La V85 necesita velocidades individuales: usa un detector E1 o la salida FCD.");
        if (!bus.empty()) {
            row(out, "Carril bus", toString((int)bus.size()), DATO, "capacidad pr\303\241ctica 60-90 autobuses/h");
        }
    }

    // ---------------- pedestrians ----------------
    const double pedsH = perHour(count(myPeds, e));
    const double pedsOp = op != nullptr ? perHour(count(myPeds, op)) : NO_DATA;
    if (sidewalks > 0 || motorEdge) {
        section(out, "PEATONES, PASOS Y SEM\303\201FOROS");
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
                        "\303\223ptimo: no hace falta (<= 12 m); Adecuado: refugio >= 1,2 m en cruces de 12-14 m; Insuficiente: > 14 m sin refugio", "", "IVP");
                    row(out, "  Ancho del paso *", num(cl->getWidth(), 1) + " m", cl->getWidth() < 3.95 ? INSUF : cl->getWidth() > 4.05 ? OPT : ADEC,
                        "Adecuado 4 m (IVP Madrid); \303\223ptimo > 4 m con mucha afluencia; Insuficiente < 4 m", "", "IVP");
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
                            "Necesario " + num(req) + " s (longitud / 0,5 m/s, Orden TMA/851/2021); \303\223ptimo " + num(req + 6) + " s (Comunidad de Madrid: + 3 s + 3 s)",
                            st.actuated ? "Sem\303\241foro actuado: verde del programa." : "", "TMA, CAM");
                        row(out, "  Rojo peatonal (espera) *", num(st.maxRed) + " s", st.maxRed > 60 ? INSUF : st.maxRed > 40 ? ADEC : OPT,
                            "\303\223ptimo <= 40 s; Adecuado <= 60 s; Insuficiente > 60 s (incita a cruzar en rojo). Ciclo " + num(st.cycle) + " s", "", "CALMAR");
                        const bool just = vehBoth > 1000 || pedBoth > 150;
                        row(out, "  Paso cebra o semaforizado", "semaforizado", ADEC, "Semaforizar si > 150 peatones/h durante 8 h o > 1.000 veh/h",
                            just ? "El sem\303\241foro est\303\241 justificado por la intensidad." : "Con " + num(vehBoth) + " veh/h y " + num(pedBoth) + " peatones/h podr\303\255a bastar un paso de cebra en una calle templada.", "IVP");
                    } else {
                        const bool bad = vehBoth > 1000 || pedBoth > 450 || lanesBoth > 4;
                        row(out, "  Paso cebra o semaforizado", "paso de cebra", bad ? INSUF : OPT,
                            "\303\223ptimo: cebra en calles templadas con tr\303\241fico y peatones bajos o medios; Insuficiente: cebra con > 1.000-1.200 veh/h, > 400-500 peatones/h o m\303\241s de 4 carriles",
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
                    "\303\223ptimo 50-80 m (<= 75 m en \303\241reas centrales); Adecuado <= 100 m; Insuficiente > 100 m",
                    "Distancia por la calle entre los dos pasos que encierran este tramo.", "CALMAR, IVP");
            } else {
                row(out, "Distancia entre pasos de peatones *", "> " + num(tot) + " m", tot > 100 ? INSUF : ND, "Adecuado <= 100 m",
                    std::string("No hay paso de peatones en ") + (cf || cb ? "uno de los extremos" : "ning\303\272n extremo") + " a lo largo de la calle.", "CALMAR, IVP");
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
        section(out, "TRANSPORTE P\303\232BLICO");
        if (has(busH) && busH > 0) {
            const double headway = 60. / busH;
            k.busHeadway = headway;
            row(out, "Intervalo de paso del autob\303\272s *", num(headway, 1) + " min", headway < 10 ? OPT : headway <= 20 ? ADEC : INSUF,
                num(busH, 1) + " autobuses/h. \303\223ptimo < 10 min (A); Adecuado 10-20 (B-C); Insuficiente > 20 (D-F)",
                "Todas las l\303\255neas que pasan por el tramo juntas. Espera media " + num(headway / 2, 1) + " min.", "TCQSM");
            row(out, "Oferta de plazas", num(busH * 90) + " plazas/h", DATO, "autobuses/h x 90 plazas (autob\303\272s est\303\241ndar de 12 m: 70-95)");
            const double vb = value(myBuses, e, SUMO_ATTR_SPEED);
            if (has(vb) && vb > 0) {
                row(out, "Velocidad del autob\303\272s en el tramo *", num(vb * 3.6, 1) + " km/h", vb * 3.6 > 15 ? OPT : vb * 3.6 >= 12 ? ADEC : INSUF,
                    "\303\223ptimo > 15 km/h; Adecuado 12-15; Insuficiente < 10-12 (centro congestionado)",
                    "Incluye paradas y sem\303\241foros de este tramo; la velocidad comercial del prontuario es la de toda la l\303\255nea.", "pr\303\241ctica t\303\251cnica");
                if (has(speed) && speed > 0) {
                    const double comp = speed / vb;
                    row(out, "Competitividad en el tramo *", num(comp, 2), comp < 1.4 ? OPT : comp <= 2.2 ? ADEC : INSUF,
                        "tiempo en bus / tiempo en coche. \303\223ptimo < 1,4; Adecuado 1,4-2,2; Insuficiente > 2,2",
                        "El prontuario lo mide puerta a puerta; aqu\303\255 solo se compara este tramo.", "IDAE");
                }
            }
            if (!bus.empty()) {
                row(out, "Carga del carril bus", num(busH, 1) + " autobuses/h", busH > 90 ? INSUF : busH >= 60 ? ADEC : OPT, "Capacidad pr\303\241ctica 60-90 autobuses/h");
            }
        } else {
            row(out, "Autobuses", "ninguno", DATO, "", "Ning\303\272n autob\303\272s ha recorrido este tramo.");
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
            section(out, "PERSONAS QUE PASAN POR LA SECCI\303\223N");
            row(out, "Reparto en esta secci\303\263n", num(tot) + " personas/h", DATO,
                "coche " + num(pc / tot * 100) + " %; autob\303\272s " + num(pb / tot * 100) + " %; bici " + num(pk / tot * 100) + " %; a pie " + num(pp / tot * 100) + " %",
                "Supuestos: 1,25 personas por coche y 30 viajeros por autob\303\272s. El umbral del prontuario (coche < 25 %, \303\263ptimo < 10 %) se refiere a los viajes internos de toda la ciudad.", "OMM, AEUB");
        }
    }

    // ---------------- environment ----------------
    section(out, "RUIDO Y EMISIONES");
    const double noise = value(myNoise, e, SUMO_ATTR_NOISE);
    if (has(noise) && noise > 0) {
        k.noise = noise;
        row(out, "Ruido del tr\303\241fico", num(noise, 1) + " dB(A)", noise > 65 ? INSUF : noise > 53 ? ADEC : OPT,
            "\303\223ptimo <= 53 dB (OMS, Lden); Adecuado <= 65 dB (RD 1367/2007, Ld residencial); Insuficiente > 65 dB",
            "Harmonoise da el ruido emitido junto a la v\303\255a, no el Lden en fachada: sirve para comparar calles y escenarios.", "RD1367, OMS");
    } else {
        row(out, "Ruido del tr\303\241fico", "-", ND, "", "Sin veh\303\255culos en el tramo.");
    }
    const double co2 = value(myEmissions, e, SUMO_ATTR_CO2_ABS);
    if (has(co2) && co2 > 0) {
        const double kgh = co2 / 1e6 / hours;
        k.co2 = kgh;
        const double dist = value(myMotor, e, SUMO_ATTR_DISTANCE);
        const double gkm = has(dist) && dist > 0 ? (co2 / 1000.) / (dist / 1000.) : NO_DATA;
        row(out, "CO2 emitido", num(kgh, 2) + " kg/h", DATO, has(gkm) ? num(gkm) + " g por veh\303\255culo y km; unos " + num(gkm / 1.25) + " g por viajero y km (1,25 personas por coche)" : "");
        const double nox = value(myEmissions, e, SUMO_ATTR_NOX_ABS);
        const double pmx = value(myEmissions, e, SUMO_ATTR_PMX_ABS);
        row(out, "NOx y part\303\255culas (PMx)", num(has(nox) ? nox / 1000 / hours : NO_DATA, 1) + " g/h / " + num(has(pmx) ? pmx / 1000 / hours : NO_DATA, 2) + " g/h", DATO, "",
            "El prontuario valora el aire con concentraciones (\302\265g/m\302\263) medidas en estaciones; SUMO calcula emisiones en gramos. Sirven para comparar, no para comprobar l\303\255mites legales.");
    }
}


std::string
MSProntuario::report(const MSEdge* edge) {
    if (!active()) {
        return "Los indicadores del prontuario no est\303\241n activos (opci\303\263n --prontuario).";
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


// ===========================================================================
// tables, Excel, field counts
// ===========================================================================
std::vector<MSProntuario::Row>
MSProntuario::rows(const MSEdge* edge, std::vector<std::string>& header) {
    std::vector<Row> result;
    header.clear();
    if (!active() || edge == nullptr || !edge->isNormal()) {
        return result;
    }
    Key k;
    std::vector<std::string> lines;
    gRows = &result;
    gSection.clear();
    compute(edge, k, &lines);
    gRows = nullptr;
    // title lines up to the first empty line after the header
    for (int i = 0; i < (int)lines.size() && i < 7; i++) {
        if (!lines[i].empty()) {
            header.push_back(lines[i]);
        }
    }
    return result;
}


std::vector<MSProntuario::Variable>
MSProntuario::variables(const MSEdge* edge) {
    std::vector<Variable> result;
    if (!active() || edge == nullptr || !edge->isNormal()) {
        return result;
    }
    const MSMeanData* md = myCars;
    auto v = [&](SumoXMLAttr a) {
        return value(md, edge, a);
    };
    auto add = [&](const std::string & attr, const std::string & es, const std::string & mide, const std::string & unidad,
    const std::string & sirve, const std::string & cifras, double val, int dec, const std::string & text = "") {
        Variable var;
        var.attr = attr;
        var.nameES = es;
        var.measures = mide;
        var.unit = unidad;
        var.purpose = sirve;
        var.reference = cifras;
        var.hasValue = has(val);
        var.value = var.hasValue ? val : 0;
        var.valueText = !text.empty() ? text : (var.hasValue ? num(val, dec) : "-");
        result.push_back(var);
    };
    const double hours = elapsed() / 3600.;
    const double entered = v(SUMO_ATTR_ENTERED), departed = v(SUMO_ATTR_DEPARTED);
    const double speed = v(SUMO_ATTR_SPEED);
    add("entered", "veh\303\255culos entrados", "Coches que entraron en la calle desde otra calle", "veh\303\255culos", "Aforo",
        "Calle residencial: 10-150. Calle de barrio con paso: 150-500. Avenida: 500-1.500 por sentido. En el ejemplo, la m\303\241s cargada tiene 185", entered, 0);
    add("departed", "veh\303\255culos que inician viaje", "Coches que empezaron su viaje en esa calle (salen de un garaje o aparcamiento)", "veh\303\255culos", "Aforo y calles que generan viajes",
        "Casi siempre 0-5 por calle; m\303\241s de 10 indica una zona con muchos or\303\255genes (viviendas, aparcamientos)", departed, 0);
    add("arrived", "veh\303\255culos que terminan viaje", "Coches que terminaron su viaje en esa calle", "veh\303\255culos", "Calles que atraen viajes (destinos)",
        "Casi siempre 0-5 por calle; m\303\241s de 10 indica un destino importante (centro comercial, instituto, estaci\303\263n)", v(SUMO_ATTR_ARRIVED), 0);
    add("left", "veh\303\255culos salidos", "Coches que salieron de la calle hacia otra", "veh\303\255culos", "Comprobar el aforo",
        "Debe ser aprox. entered + departed - arrived. Si es bastante menor, quedan coches atrapados en la calle al acabar", v(SUMO_ATTR_LEFT), 0);
    add("flow", "intensidad (flujo)", "Intensidad calculada como densidad x velocidad (no es un recuento)", "veh\303\255culos/hora", "Aforo aproximado con tr\303\241fico fluido",
        "Un carril urbano con sem\303\241foro admite unos 600-900 veh/h; sin sem\303\241foro, hasta unos 1.800. Si flow es mucho menor que entered + departed (por hora), hay atasco (ejemplo: 69 frente a 110)", v(SUMO_ATTR_FLOW), 2,
        has(v(SUMO_ATTR_FLOW)) ? num(v(SUMO_ATTR_FLOW), 2) + "  (entered + departed por hora: " + num(hours > 0 && has(entered) ? (entered + (has(departed) ? departed : 0)) / hours : NO_DATA, 2) + ")" : "");
    add("speed", "velocidad media", "Velocidad media de los coches en la calle", "km/h", "Atasco",
        "Calle a 50 km/h fluida: 30-47 km/h. Calle a 30 km/h fluida: 18-29 km/h. Menos de 7 km/h: atasco. Ejemplo: 34,60 fluida, 1,37 atascada", has(speed) ? speed * 3.6 : NO_DATA, 2);
    add("speedRelative", "velocidad relativa", "Velocidad media dividida entre el l\303\255mite de la calle (1 = van al l\303\255mite)", "proporci\303\263n de 0 a 1", "Atasco: el indicador m\303\241s claro",
        "M\303\241s de 0,7: fluido. 0,4-0,7: denso. Menos de 0,4: atasco. Puede pasar de 1 porque algunos conductores superan algo el l\303\255mite", v(SUMO_ATTR_SPEEDREL), 2);
    add("occupancy", "ocupaci\303\263n", "Porcentaje de la calle ocupado por coches", "%", "Atasco",
        "Menos de 10 %: fluido. 10-25 %: denso. M\303\241s de 25 %: atasco. Ejemplo: 1,3 % fluida, 42,5 % atascada", v(SUMO_ATTR_OCCUPANCY), 2);
    add("density", "densidad", "Coches que hay de media por kil\303\263metro de calle", "veh/km", "Atasco",
        "Menos de 20 por carril: fluido. 20-50: denso. M\303\241s de 50: atasco. 133 veh/km es el m\303\241ximo f\303\255sico: coches parados en fila, uno cada 7,5 m (5 m de coche + 2,5 m de hueco)", v(SUMO_ATTR_DENSITY), 2);
    add("laneDensity", "densidad por carril", "Igual que density, pero por carril", "veh/km y carril", "Atasco en calles de varios carriles",
        "Mismas cifras que density. En calles de un carril, density y laneDensity coinciden", v(SUMO_ATTR_LANEDENSITY), 2);
    const double nVeh = has(entered) ? entered + (has(departed) ? departed : 0) : 0;
    const double wt = v(SUMO_ATTR_WAITINGTIME), tl = v(SUMO_ATTR_TIMELOSS);
    add("waitingTime", "tiempo de espera (parado)", "Suma de los segundos que han estado parados todos los coches", "s (total)", "Atasco y colas en sem\303\241foros",
        "Div\303\255delo entre los coches. Por coche: 0-5 s fluido; 10-30 s normal con sem\303\241foro; m\303\241s de 30-45 s atasco", wt, 2,
        has(wt) ? num(wt, 2) + "  (por coche: " + (nVeh > 0 ? num(wt / nVeh, 2) : std::string("-")) + " s)" : "");
    add("timeLoss", "tiempo perdido (demora)", "Suma de los segundos perdidos por no ir a la velocidad deseada (frenazos, colas)", "s (total)", "Coste del atasco",
        "Div\303\255delo entre los coches. Por coche y calle: menos de 5 s fluido; 10-30 s con sem\303\241foro; m\303\241s de 30 s atasco", tl, 2,
        has(tl) ? num(tl, 2) + "  (por coche: " + (nVeh > 0 ? num(tl / nVeh, 2) : std::string("-")) + " s)" : "");
    const double tt = v(SUMO_ATTR_TRAVELTIME);
    double vfree = 0;
    for (const MSLane* l : edge->getLanes()) {
        if (isCarLane(l)) {
            vfree = MAX2(vfree, l->getSpeedLimit());
        }
    }
    add("traveltime", "tiempo de recorrido", "Tiempo medio que tarda un coche en recorrer la calle", "s", "Comparar escenarios",
        "Comp\303\241ralo con el tiempo sin tr\303\241fico: longitud / l\303\255mite (100 m a 50 km/h = 7 s). Hasta 2 veces ese valor: normal. M\303\241s de 3 veces: atasco", tt, 2,
        has(tt) && tt > 0 ? num(tt, 2) + "  (sin tr\303\241fico: " + (vfree > 0 ? num(edge->getLength() / vfree, 2) : std::string("-")) + " s)" : "");
    const double ss = v(SUMO_ATTR_SAMPLEDSECONDS);
    add("sampledSeconds", "segundos-veh\303\255culo", "Suma de los segundos que han pasado todos los coches en la calle", "veh x s", "Horas de veh\303\255culo en la calle (/ 3600)",
        "/ 3600 = coches que hay a la vez en la calle de media. Comp\303\241ralo con lo que cabe en la calle en cola (longitud / 7,5 m): si se acerca, la calle est\303\241 llena", ss, 2,
        has(ss) ? num(ss, 2) + "  (coches a la vez: " + num(elapsed() > 0 ? ss / elapsed() : NO_DATA, 2) + "; caben " + num(edge->getLength() / 7.5, 1) + ")" : "");
    add("distance", "distancia recorrida", "Metros recorridos por todos los coches en la calle", "metros", "Kil\303\263metros totales recorridos (/ 1000)",
        "Aprox. coches x longitud de la calle. Sumado en toda la zona del ejemplo: 372 km en 1 hora", v(SUMO_ATTR_DISTANCE), 2);
    const double lcf = v(SUMO_ATTR_LANECHANGEDFROM), lct = v(SUMO_ATTR_LANECHANGEDTO);
    add("laneChangedFrom / To", "cambios de carril (desde / hacia)", "Cambios de carril hechos desde / hacia la calle", "n.\302\272 de maniobras", "Calles con muchas maniobras de incorporaci\303\263n",
        "0 en calles de un carril. Decenas en avenidas de varios carriles. Muchos cientos: zona de incorporaci\303\263n conflictiva", lcf, 0, num(lcf, 0) + " / " + num(lct, 0));
    const double ott = v(SUMO_ATTR_OVERLAPTRAVELTIME), od = v(SUMO_ATTR_OVERLAPDENSITY);
    add("overlapTraveltime / overlapDensity", "tiempo de recorrido y densidad con solape", "Como traveltime y density, pero contando tambi\303\251n el trozo del coche que asoma en la calle siguiente", "s, veh/km",
        "Uso avanzado; se pueden ignorar", "Algo mayores que traveltime y density (ejemplo: 20,85 frente a 20,01 s)", ott, 2, num(ott, 2) + " / " + num(od, 2));
    return result;
}


std::string
MSProntuario::variablesReport(const MSEdge* edge) {
    if (!active() || edge == nullptr || !edge->isNormal()) {
        return "Elige una calle normal con la simulaci\303\263n cargada.";
    }
    std::vector<std::string> out;
    out.push_back("QU\303\211 SIGNIFICA CADA VARIABLE (datos edgeData de los coches)");
    out.push_back("");
    out.push_back("Calle: " + (edge->getStreetName().empty() ? std::string("(sin nombre)") : edge->getStreetName()) + "    tramo " + edge->getID());
    out.push_back("Datos: coches, toda la simulaci\303\263n hasta ahora (" + num(elapsed() / 60.) + " min). Velocidades en km/h.");
    for (const Variable& var : variables(edge)) {
        out.push_back("");
        out.push_back("  " + var.attr + " = " + var.nameES + ":  " + var.valueText + "  [" + var.unit + "]");
        wrap(out, "Qu\303\251 mide: " + var.measures + ". Para qu\303\251 sirve: " + var.purpose + ".", "      ");
        wrap(out, "Cifras orientativas (calle urbana, 1 hora): " + var.reference, "      ");
    }
    out.push_back("");
    wrap(out, "Las cifras orientativas sirven para tener un orden de magnitud en calles urbanas. No son normas oficiales y cambian seg\303\272n el tipo de calle, los sem\303\241foros y la hora.", "");
    std::string r;
    for (const std::string& l : out) {
        r += l + "\n";
    }
    return r;
}


bool
MSProntuario::exportExcel(const MSEdge* edge, const std::string& file, bool indicators, bool vars, std::string& error) {
    if (!active() || edge == nullptr || !edge->isNormal()) {
        error = "No hay datos de esta calle.";
        return false;
    }
    XLSXWriter w;
    const std::string calle = edge->getStreetName().empty() ? std::string("(sin nombre)") : edge->getStreetName();
    if (indicators) {
        std::vector<std::string> header;
        const std::vector<Row> rs = rows(edge, header);
        std::vector<XLSXWriter::Row> sheet;
        sheet.push_back({XLSXWriter::Cell("Indicadores del Prontuario de urbanismo y movilidad", XLSXWriter::TITLE)});
        for (int i = 1; i < (int)header.size(); i++) {
            sheet.push_back({XLSXWriter::Cell(header[i])});
        }
        sheet.push_back({});
        sheet.push_back({XLSXWriter::Cell("Bloque", XLSXWriter::HEADER), XLSXWriter::Cell("Indicador", XLSXWriter::HEADER),
                         XLSXWriter::Cell("Valor", XLSXWriter::HEADER), XLSXWriter::Cell("Valoraci\303\263n", XLSXWriter::HEADER),
                         XLSXWriter::Cell("Umbrales", XLSXWriter::HEADER), XLSXWriter::Cell("Observaciones", XLSXWriter::HEADER),
                         XLSXWriter::Cell("Fuente", XLSXWriter::HEADER)});
        for (const Row& r : rs) {
            XLSXWriter::Style gs = r.gradeCode == 0 ? XLSXWriter::GOOD : r.gradeCode == 1 ? XLSXWriter::FAIR : r.gradeCode == 2 ? XLSXWriter::BAD : XLSXWriter::WRAP;
            sheet.push_back({XLSXWriter::Cell(r.section, XLSXWriter::WRAP), XLSXWriter::Cell(r.name, XLSXWriter::WRAP), XLSXWriter::Cell(r.value, XLSXWriter::WRAP),
                             XLSXWriter::Cell(r.grade, gs), XLSXWriter::Cell(r.umbral, XLSXWriter::WRAP), XLSXWriter::Cell(r.note, XLSXWriter::WRAP),
                             XLSXWriter::Cell(r.source, XLSXWriter::WRAP)});
        }
        w.addSheet("Indicadores", sheet, {24, 34, 20, 14, 60, 60, 18});
    }
    if (vars) {
        std::vector<XLSXWriter::Row> sheet;
        sheet.push_back({XLSXWriter::Cell("Qu\303\251 significa cada variable: " + calle + " (tramo " + edge->getID() + ")", XLSXWriter::TITLE)});
        sheet.push_back({XLSXWriter::Cell("Datos edgeData de los coches, toda la simulaci\303\263n (" + num(elapsed() / 60.) + " min). Velocidades en km/h.")});
        sheet.push_back({});
        sheet.push_back({XLSXWriter::Cell("Variable", XLSXWriter::HEADER), XLSXWriter::Cell("Variable (castellano)", XLSXWriter::HEADER),
                         XLSXWriter::Cell("Qu\303\251 mide", XLSXWriter::HEADER), XLSXWriter::Cell("Unidad", XLSXWriter::HEADER),
                         XLSXWriter::Cell("Para qu\303\251 sirve", XLSXWriter::HEADER),
                         XLSXWriter::Cell("Cifras orientativas (calle urbana, intervalo de 1 hora)", XLSXWriter::HEADER),
                         XLSXWriter::Cell("Valor en la simulaci\303\263n", XLSXWriter::HEADER), XLSXWriter::Cell("Detalle", XLSXWriter::HEADER)});
        for (const Variable& var : variables(edge)) {
            XLSXWriter::Row r = {XLSXWriter::Cell(var.attr, XLSXWriter::WRAP), XLSXWriter::Cell(var.nameES, XLSXWriter::WRAP),
                                 XLSXWriter::Cell(var.measures, XLSXWriter::WRAP), XLSXWriter::Cell(var.unit, XLSXWriter::WRAP),
                                 XLSXWriter::Cell(var.purpose, XLSXWriter::WRAP), XLSXWriter::Cell(var.reference, XLSXWriter::WRAP)
                                };
            if (var.hasValue) {
                r.push_back(XLSXWriter::Cell(var.value));
            } else {
                r.push_back(XLSXWriter::Cell("-", XLSXWriter::WRAP));
            }
            r.push_back(XLSXWriter::Cell(var.valueText, XLSXWriter::WRAP));
            sheet.push_back(r);
        }
        sheet.push_back({});
        sheet.push_back({XLSXWriter::Cell("Las cifras orientativas sirven para tener un orden de magnitud en calles urbanas. No son normas oficiales y cambian seg\303\272n el tipo de calle, los sem\303\241foros y la hora.")});
        w.addSheet("Variables", sheet, {22, 26, 40, 14, 28, 60, 16, 34});
    }
    return w.save(file, error);
}


std::string
MSProntuario::scenarioDir() {
    const OptionsCont& oc = OptionsCont::getOptions();
    std::string f;
    if (oc.isSet("configuration-file")) {
        f = oc.getString("configuration-file");
    } else if (oc.isSet("net-file")) {
        f = oc.getString("net-file");
    }
    return f.empty() ? "" : FileHelpers::getFilePath(f);
}


std::string
MSProntuario::fieldCountFile() {
    return scenarioDir() + "aforos_campo.xml";
}


void
MSProntuario::loadFieldCounts() {
    myFieldCounts.clear();
    std::ifstream in(XMLSubSys::transcodeToLocal(fieldCountFile()).c_str());
    if (!in.good()) {
        return;
    }
    const std::regex re("<edge\\s+id=\"([^\"]+)\"[^>]*\\baforo=\"([0-9.]+)\"(?:[^>]*\\bbuses=\"([0-9.]+)\")?");
    std::string line;
    while (std::getline(in, line)) {
        std::smatch m;
        if (std::regex_search(line, m, re)) {
            myFieldCounts[m[1].str()] = std::make_pair(StringUtils::toDouble(m[2].str()), m[3].matched ? StringUtils::toDouble(m[3].str()) : 0.);
        }
    }
}


bool
MSProntuario::writeFieldCounts(std::string& error) {
    const OptionsCont& oc = OptionsCont::getOptions();
    const double begin = STEPS2TIME(string2time(oc.getString("begin")));
    double end = oc.isSet("end") ? STEPS2TIME(string2time(oc.getString("end"))) : -1;
    if (end <= begin) {
        end = begin + 3600;
    }
    const double hours = (end - begin) / 3600.;
    std::ofstream out(XMLSubSys::transcodeToLocal(fieldCountFile()).c_str());
    if (!out.good()) {
        error = "No se puede escribir '" + fieldCountFile() + "'.";
        return false;
    }
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        << "<!-- Aforos de campo introducidos en sumo-gui (veh/h de veh\303\255culos a motor).\n"
        << "     aforo: veh/h contados; buses: autobuses/h simulados en la calle al introducir el aforo;\n"
        << "     entered: coches que debe reproducir routeSampler en el intervalo = (aforo - buses) x horas. -->\n"
        << "<data>\n"
        << "    <interval id=\"aforos_campo\" begin=\"" << begin << "\" end=\"" << end << "\">\n";
    for (const auto& item : myFieldCounts) {
        const double cars = MAX2(0., item.second.first - item.second.second) * hours;
        out << "        <edge id=\"" << item.first << "\" aforo=\"" << item.second.first << "\" buses=\"" << item.second.second
            << "\" entered=\"" << (int)std::round(cars) << "\"/>\n";
    }
    out << "    </interval>\n</data>\n";
    out.close();
    return true;
}


double
MSProntuario::fieldCount(const MSEdge* edge) {
    auto it = myFieldCounts.find(edge->getID());
    return it == myFieldCounts.end() ? -1 : it->second.first;
}


bool
MSProntuario::setFieldCount(const MSEdge* edge, double vehPerHour, std::string& error) {
    if (vehPerHour < 0) {
        myFieldCounts.erase(edge->getID());
    } else {
        const double busH = elapsed() > 0 ? value(myBuses, edge, SUMO_ATTR_ENTERED) : NO_DATA;
        const double busD = elapsed() > 0 ? value(myBuses, edge, SUMO_ATTR_DEPARTED) : NO_DATA;
        const double buses = has(busH) ? (busH + (has(busD) ? busD : 0)) / (elapsed() / 3600.) : 0;
        myFieldCounts[edge->getID()] = std::make_pair(vehPerHour, buses);
    }
    return writeFieldCounts(error);
}


int
MSProntuario::numFieldCounts() {
    return (int)myFieldCounts.size();
}


std::string
MSProntuario::fringeInfo(const MSEdge* e, double& factor) {
    factor = -1;
    // factor used by osmWebWizard (build.bat / build.sh of the scenario)
    for (const std::string name : {
                "build.bat", "build.sh"
            }) {
        std::ifstream in(XMLSubSys::transcodeToLocal(scenarioDir() + name).c_str());
        std::string line;
        while (in.good() && std::getline(in, line)) {
            const std::string::size_type p = line.find("--fringe-factor");
            if (p != std::string::npos && (factor < 0 || line.find("passenger") != std::string::npos)) {
                std::istringstream is(line.substr(p + 15));
                double f;
                if (is >> f) {
                    factor = f;
                }
            }
        }
        if (factor > 0) {
            break;
        }
    }
    const MSEdge* op = opposite(e);
    bool entry = true, exit = true;
    for (const MSEdge* x : e->getFromJunction()->getIncoming()) {
        if (x->isNormal() && x != op) {
            entry = false;
        }
    }
    for (const MSEdge* x : e->getToJunction()->getOutgoing()) {
        if (x->isNormal() && x != op) {
            exit = false;
        }
    }
    if (entry) {
        return "s\303\255, entrada a la zona";
    }
    if (exit) {
        return "s\303\255, salida de la zona";
    }
    return "no, calle interior";
}


/****************************************************************************/
