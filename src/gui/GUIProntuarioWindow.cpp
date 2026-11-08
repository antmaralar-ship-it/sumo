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
/// @file    GUIProntuarioWindow.cpp
/// @date    November 2026
///
// Window with the prontuario indicators (or the edgeData variables) of one street
/****************************************************************************/
#include <config.h>

#include <cstdlib>
#include <fstream>
#include <vector>
#include <microsim/MSNet.h>
#include <microsim/MSEdge.h>
#include <microsim/output/MSProntuario.h>
#include <utils/common/StringUtils.h>
#include <utils/foxtools/MFXUtils.h>
#include <utils/options/OptionsCont.h>
#include <utils/xml/XMLSubSys.h>
#include <utils/gui/windows/GUIMainWindow.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/images/GUIIconSubSys.h>
#include <utils/gui/div/GUIUserIO.h>
#include <utils/gui/div/GUIIOGlobals.h>
#include "GUIApplicationWindow.h"
#include "GUIRunThread.h"
#include "GUIProntuarioWindow.h"


// ===========================================================================
// FOX callback mapping
// ===========================================================================
FXDEFMAP(GUIProntuarioWindow) GUIProntuarioWindowMap[] = {
    FXMAPFUNC(SEL_COMMAND, GUIProntuarioWindow::MID_PRONTUARIO_REFRESH, GUIProntuarioWindow::onCmdRefresh),
    FXMAPFUNC(SEL_COMMAND, GUIProntuarioWindow::MID_PRONTUARIO_COPY,    GUIProntuarioWindow::onCmdCopy),
    FXMAPFUNC(SEL_COMMAND, GUIProntuarioWindow::MID_PRONTUARIO_EXPORT,  GUIProntuarioWindow::onCmdExport),
    FXMAPFUNC(SEL_COMMAND, GUIProntuarioWindow::MID_PRONTUARIO_CLOSE,   GUIProntuarioWindow::onCmdClose),
};

FXIMPLEMENT(GUIProntuarioWindow, FXMainWindow, GUIProntuarioWindowMap, ARRAYNUMBER(GUIProntuarioWindowMap))


// ===========================================================================
// helpers
// ===========================================================================
namespace {

/// @brief locks the simulation while reading from it
class SimulationLock {
public:
    SimulationLock(GUIMainWindow* app) {
        GUIApplicationWindow* aw = dynamic_cast<GUIApplicationWindow*>(app);
        myRunner = aw != nullptr ? aw->getRunner() : nullptr;
        if (myRunner != nullptr) {
            myRunner->getSimulationLock().lock();
        }
    }
    ~SimulationLock() {
        if (myRunner != nullptr) {
            myRunner->getSimulationLock().unlock();
        }
    }
private:
    GUIRunThread* myRunner;
};

std::string quote(const std::string& s) {
    return "\"" + s + "\"";
}

} // namespace


// ===========================================================================
// method definitions
// ===========================================================================
GUIProntuarioWindow::GUIProntuarioWindow(GUIMainWindow& app, const std::string& edgeID, bool variables) :
    FXMainWindow(app.getApp(), ((variables ? "Variables de la calle: " : "Prontuario: ") + edgeID).c_str(),
                 GUIIconSubSys::getIcon(GUIIcon::APP_TABLE), nullptr, DECOR_ALL, 60, 60, 900, 680),
    myApp(&app),
    myEdgeID(edgeID),
    myVariables(variables) {
    FXVerticalFrame* frame = new FXVerticalFrame(this, LAYOUT_FILL_X | LAYOUT_FILL_Y);
    FXHorizontalFrame* bar = new FXHorizontalFrame(frame, LAYOUT_FILL_X);
    new FXButton(bar, "Actualizar\tVolver a calcular con los datos de la simulaci\303\263n hasta ahora", nullptr, this, MID_PRONTUARIO_REFRESH, BUTTON_NORMAL);
    new FXButton(bar, "Copiar\tCopiar el texto al portapapeles para pegarlo en un documento", nullptr, this, MID_PRONTUARIO_COPY, BUTTON_NORMAL);
    new FXButton(bar, "Exportar a Excel...\tGuardar la tabla en un archivo .xlsx", nullptr, this, MID_PRONTUARIO_EXPORT, BUTTON_NORMAL);
    new FXButton(bar, "Cerrar", nullptr, this, MID_PRONTUARIO_CLOSE, BUTTON_NORMAL | LAYOUT_RIGHT);
    myText = new FXText(frame, nullptr, 0, TEXT_READONLY | LAYOUT_FILL_X | LAYOUT_FILL_Y);
    // a fixed-width font keeps the values aligned; created once and shared by all windows
    static FXFont* font = nullptr;
    if (font == nullptr) {
        font = new FXFont(app.getApp(), "Courier New", 10);
        font->create();
    }
    myText->setFont(font);
    refresh();
    myApp->addChild(this);
    create();
    show();
}


GUIProntuarioWindow::~GUIProntuarioWindow() {
    myApp->removeChild(this);
}


void
GUIProntuarioWindow::refresh() {
    if (!MSNet::hasInstance() || !MSProntuario::active()) {
        myReport = "No hay ninguna simulaci\303\263n cargada con los indicadores del prontuario.";
    } else {
        SimulationLock lock(myApp);
        const MSEdge* edge = MSEdge::dictionary(myEdgeID);
        if (edge == nullptr) {
            myReport = "El tramo '" + myEdgeID + "' ya no existe en la red cargada.";
        } else {
            myReport = myVariables ? MSProntuario::variablesReport(edge) : MSProntuario::report(edge);
        }
    }
    myText->setText(myReport.c_str());
}


long
GUIProntuarioWindow::onCmdRefresh(FXObject*, FXSelector, void*) {
    refresh();
    return 1;
}


long
GUIProntuarioWindow::onCmdCopy(FXObject*, FXSelector, void*) {
    GUIUserIO::copyToClipboard(*getApp(), myReport);
    return 1;
}


long
GUIProntuarioWindow::onCmdExport(FXObject*, FXSelector, void*) {
    exportExcel(this, myApp, myEdgeID, !myVariables, myVariables);
    return 1;
}


long
GUIProntuarioWindow::onCmdClose(FXObject*, FXSelector, void*) {
    close(true);
    return 1;
}


void
GUIProntuarioWindow::exportExcel(FXWindow* parent, GUIMainWindow* app, const std::string& edgeID, bool indicators, bool variables) {
    FXString file = MFXUtils::getFilename2Write(parent, "Exportar a Excel", "Libro de Excel (*.xlsx)",
                    GUIIconSubSys::getIcon(GUIIcon::SAVE), gCurrentFolder);
    if (file == "") {
        return;
    }
    std::string f = file.text();
    if (!StringUtils::endsWith(StringUtils::to_lower_case(f), ".xlsx")) {
        f += ".xlsx";
    }
    std::string error;
    bool ok;
    {
        SimulationLock lock(app);
        const MSEdge* edge = MSEdge::dictionary(edgeID);
        ok = edge != nullptr && MSProntuario::exportExcel(edge, f, indicators, variables, error);
        if (edge == nullptr) {
            error = "El tramo '" + edgeID + "' ya no existe.";
        }
    }
    if (ok) {
        FXMessageBox::information(parent, MBOX_OK, "Exportar a Excel", "%s", ("Guardado en\n" + f).c_str());
    } else {
        FXMessageBox::error(parent, MBOX_OK, "Exportar a Excel", "%s", error.c_str());
    }
}


void
GUIProntuarioWindow::adjustDemand(FXWindow* parent, GUIMainWindow* app) {
    const OptionsCont& oc = OptionsCont::getOptions();
    if (!oc.isSet("configuration-file")) {
        FXMessageBox::error(parent, MBOX_OK, "Ajustar la demanda", "%s",
                            "Para ajustar la demanda hay que abrir el escenario con su archivo .sumocfg (Archivo > Abrir simulaci\303\263n).");
        return;
    }
    const char* home = std::getenv("SUMO_HOME");
    if (home == nullptr || std::string(home).empty()) {
        FXMessageBox::error(parent, MBOX_OK, "Ajustar la demanda", "%s",
                            "No est\303\241 definida la variable SUMO_HOME. Vuelve a instalar SUMO o def\303\255nela apuntando a la carpeta de SUMO.");
        return;
    }
    const std::string cfg = oc.getString("configuration-file");
    const std::string msg = "Se van a ajustar los viajes de coche del escenario a los "
                            + toString(MSProntuario::numFieldCounts()) + " aforos de campo introducidos.\n\n"
                            "1. SUMO calcula las rutas de los viajes de coche actuales.\n"
                            "2. routeSampler quita viajes donde sobran coches y a\303\261ade donde faltan,\n"
                            "   conservando el tr\303\241fico de las calles sin aforo.\n"
                            "3. El escenario pasa a usar la nueva demanda (se guarda una copia\n"
                            "   del .sumocfg original con la terminaci\303\263n .antes_aforos)\n"
                            "   y la simulaci\303\263n se vuelve a cargar.\n\n"
                            "Hace falta Python (el mismo que usa OSM Web Wizard). \302\277Continuar?";
    if (FXMessageBox::question(parent, MBOX_YES_NO, "Ajustar la demanda a los aforos de campo", "%s", msg.c_str()) != MBOX_CLICKED_YES) {
        return;
    }
    const std::string script = std::string(home) + "/tools/prontuario/ajustar_aforos.py";
    const std::string log = MSProntuario::scenarioDir() + "prontuario_ajuste.log";
    int status = -1;
    parent->getApp()->beginWaitCursor();
    for (const std::string python : {
#ifdef WIN32
                "python", "py -3"
#else
                "python3", "python"
#endif
            }) {
        std::string cmd = python + " " + quote(script) + " -c " + quote(cfg) + " > " + quote(log) + " 2>&1";
#ifdef WIN32
        cmd = "\"" + cmd + "\"";
#endif
        status = std::system(XMLSubSys::transcodeToLocal(cmd).c_str());
        if (status == 0) {
            break;
        }
        // try the next interpreter only if this one does not exist
        std::ifstream in(XMLSubSys::transcodeToLocal(log).c_str());
        std::string first;
        std::getline(in, first);
        if (first.find("ERROR") != std::string::npos || first.find("Viajes") != std::string::npos) {
            break;
        }
    }
    parent->getApp()->endWaitCursor();
    std::ifstream in(XMLSubSys::transcodeToLocal(log).c_str());
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) {
        lines.push_back(line);
    }
    std::string tail;
    for (int i = MAX2(0, (int)lines.size() - 14); i < (int)lines.size(); i++) {
        tail += lines[i] + "\n";
    }
    if (status != 0) {
        FXMessageBox::error(parent, MBOX_OK, "Ajustar la demanda", "%s",
                            ("No se pudo ajustar la demanda. Detalles en " + log + ":\n\n" + tail).c_str());
        return;
    }
    FXMessageBox::information(parent, MBOX_OK, "Ajustar la demanda", "%s",
                              ("Demanda ajustada. La simulaci\303\263n se va a volver a cargar.\n\n" + tail).c_str());
    app->getApp()->forceRefresh();
    app->handle(app, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_R_RELOAD), nullptr);
}


/****************************************************************************/
