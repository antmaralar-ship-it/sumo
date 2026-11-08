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
// Window with the prontuario indicators of one street
/****************************************************************************/
#include <config.h>

#include <microsim/MSNet.h>
#include <microsim/MSEdge.h>
#include <microsim/output/MSProntuario.h>
#include <utils/gui/windows/GUIMainWindow.h>
#include <utils/gui/images/GUIIconSubSys.h>
#include <utils/gui/div/GUIUserIO.h>
#include "GUIApplicationWindow.h"
#include "GUIRunThread.h"
#include "GUIProntuarioWindow.h"


// ===========================================================================
// FOX callback mapping
// ===========================================================================
FXDEFMAP(GUIProntuarioWindow) GUIProntuarioWindowMap[] = {
    FXMAPFUNC(SEL_COMMAND, GUIProntuarioWindow::MID_PRONTUARIO_REFRESH, GUIProntuarioWindow::onCmdRefresh),
    FXMAPFUNC(SEL_COMMAND, GUIProntuarioWindow::MID_PRONTUARIO_COPY,    GUIProntuarioWindow::onCmdCopy),
    FXMAPFUNC(SEL_COMMAND, GUIProntuarioWindow::MID_PRONTUARIO_CLOSE,   GUIProntuarioWindow::onCmdClose),
};

FXIMPLEMENT(GUIProntuarioWindow, FXMainWindow, GUIProntuarioWindowMap, ARRAYNUMBER(GUIProntuarioWindowMap))


// ===========================================================================
// method definitions
// ===========================================================================
GUIProntuarioWindow::GUIProntuarioWindow(GUIMainWindow& app, const std::string& edgeID) :
    FXMainWindow(app.getApp(), ("Prontuario: " + edgeID).c_str(), GUIIconSubSys::getIcon(GUIIcon::APP_TABLE), nullptr, DECOR_ALL, 60, 60, 860, 680),
    myApp(&app),
    myEdgeID(edgeID) {
    FXVerticalFrame* frame = new FXVerticalFrame(this, LAYOUT_FILL_X | LAYOUT_FILL_Y);
    FXHorizontalFrame* bar = new FXHorizontalFrame(frame, LAYOUT_FILL_X);
    new FXButton(bar, "Actualizar\tVolver a calcular con los datos de la simulaci\303\263n hasta ahora", nullptr, this, MID_PRONTUARIO_REFRESH, BUTTON_NORMAL);
    new FXButton(bar, "Copiar\tCopiar la ficha al portapapeles para pegarla en un documento", nullptr, this, MID_PRONTUARIO_COPY, BUTTON_NORMAL);
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
        GUIApplicationWindow* aw = dynamic_cast<GUIApplicationWindow*>(myApp);
        GUIRunThread* runner = aw != nullptr ? aw->getRunner() : nullptr;
        if (runner != nullptr) {
            runner->getSimulationLock().lock();
        }
        const MSEdge* edge = MSEdge::dictionary(myEdgeID);
        myReport = edge != nullptr ? MSProntuario::report(edge) : "El tramo '" + myEdgeID + "' ya no existe en la red cargada.";
        if (runner != nullptr) {
            runner->getSimulationLock().unlock();
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
GUIProntuarioWindow::onCmdClose(FXObject*, FXSelector, void*) {
    close(true);
    return 1;
}


/****************************************************************************/
