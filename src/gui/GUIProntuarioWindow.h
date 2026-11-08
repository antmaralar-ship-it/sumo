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
/// @file    GUIProntuarioWindow.h
/// @date    November 2026
///
// Window with the prontuario indicators of one street
/****************************************************************************/
#pragma once
#include <config.h>

#include <string>
#include <utils/foxtools/fxheader.h>


// ===========================================================================
// class declarations
// ===========================================================================
class GUIMainWindow;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GUIProntuarioWindow
 * @brief Shows the report of MSProntuario for one edge (right click on a street)
 */
class GUIProntuarioWindow : public FXMainWindow {
    FXDECLARE(GUIProntuarioWindow)

public:
    enum {
        MID_PRONTUARIO_REFRESH = FXMainWindow::ID_LAST,
        MID_PRONTUARIO_COPY,
        MID_PRONTUARIO_CLOSE
    };

    /// @brief Constructor (the window shows itself)
    GUIProntuarioWindow(GUIMainWindow& app, const std::string& edgeID);

    /// @brief Destructor
    ~GUIProntuarioWindow();

    /// @brief recompute with the current simulation state
    long onCmdRefresh(FXObject*, FXSelector, void*);

    /// @brief copy the report to the clipboard
    long onCmdCopy(FXObject*, FXSelector, void*);

    /// @brief close the window
    long onCmdClose(FXObject*, FXSelector, void*);

protected:
    /// @brief FOX needs this
    GUIProntuarioWindow() {}

private:
    void refresh();

    GUIMainWindow* myApp = nullptr;
    std::string myEdgeID;
    FXText* myText = nullptr;
    std::string myReport;
};
