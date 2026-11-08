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
/// @file    MSProntuario.h
/// @date    November 2026
///
// Street indicators graded with the "Prontuario de urbanismo y movilidad"
// (module 1659, Movilidad segura y sostenible). Collects in-memory edge
// statistics for the whole simulation and builds a per-street report.
/****************************************************************************/
#pragma once
#include <config.h>

#include <string>
#include <vector>
#include <map>
#include <utils/common/SUMOTime.h>


// ===========================================================================
// class declarations
// ===========================================================================
class MSEdge;
class MSNet;
class MSMeanData;
class OptionsCont;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSProntuario
 * @brief Collects per-edge statistics in memory and grades them
 *
 * The collectors are registered with the detector control without an output
 * interval, so they are never written or reset and keep the totals of the
 * whole simulation, also after the simulation has ended.
 */
class MSProntuario {
public:
    /// @brief key values of one edge (used for the table output)
    struct Key {
        // missing values are -1e30
        double flow = -1e30;       // motor vehicles per hour
        double ic = -1e30;         // intensity / capacity
        double rel = -1e30;        // travel speed / free speed
        double congestion = -1e30; // congestion index in %
        double peds = -1e30;       // pedestrians per hour
        double busHeadway = -1e30; // minutes
        double noise = -1e30;      // dB(A)
        double co2 = -1e30;        // kg/h
        double geh = -1e30;        // GEH against the field count
        std::string los;        // level of service A-F
    };

    /// @brief one indicator of the report (for tables)
    struct Row {
        std::string section, name, value, grade, umbral, note, source;
        int gradeCode = 3; // 0 optimo, 1 adecuado, 2 insuficiente, 3 dato, 4 no aplica, 5 sin datos
    };

    /// @brief one variable of the edgeData table (as in the classroom tutorial)
    struct Variable {
        std::string attr, nameES, measures, unit, purpose, reference, valueText;
        double value = 0;
        bool hasValue = false;
    };

    /// @brief register the options
    static void insertOptions(OptionsCont& oc);

    /// @brief the indicators of the edge as table rows (header receives the title lines)
    static std::vector<Row> rows(const MSEdge* edge, std::vector<std::string>& header);

    /// @brief the edgeData variables of the edge (cars, whole simulation)
    static std::vector<Variable> variables(const MSEdge* edge);

    /// @brief plain text version of the variables table
    static std::string variablesReport(const MSEdge* edge);

    /// @brief write the indicators (and the variables) of the edge into an Excel workbook
    static bool exportExcel(const MSEdge* edge, const std::string& file, bool indicators, bool vars, std::string& error);

    /// @brief folder of the loaded scenario (configuration or network)
    static std::string scenarioDir();

    /// @brief file with the field counts of the scenario
    static std::string fieldCountFile();

    /// @brief field count (veh/h) entered for the edge, -1 if there is none
    static double fieldCount(const MSEdge* edge);

    /// @brief stores (or with value < 0 removes) the field count of the edge
    static bool setFieldCount(const MSEdge* edge, double vehPerHour, std::string& error);

    /// @brief number of streets with field counts
    static int numFieldCounts();

    /// @brief through traffic: whether the edge is at the border of the area and the factor used by osmWebWizard
    static std::string fringeInfo(const MSEdge* edge, double& factor);

    /// @brief build the collectors if wished (called after the network and additionals are loaded)
    static void install(const OptionsCont& oc, bool gui);

    /// @brief whether collectors exist for the current network
    static bool active();

    /// @brief the report (Spanish, plain text) for the given edge
    static std::string report(const MSEdge* edge);

    /// @brief writes --prontuario-output (called when the simulation is closed)
    static void writeOutput();

    /// @brief seconds of simulation covered by the collectors
    static double elapsed();

    /// @brief congestion index (%) of the edge since the begin, -1 if there is no data (cached per time step)
    static double congestionIndex(const MSEdge* edge);

private:
    static void compute(const MSEdge* edge, Key& key, std::vector<std::string>* out);
    static void loadFieldCounts();
    static bool writeFieldCounts(std::string& error);
    /// @brief field counts (veh/h) and simulated buses/h at the time of entering
    static std::map<std::string, std::pair<double, double> > myFieldCounts;
    static void buildCrossingMap();

    static MSNet* myNet;
    static MSMeanData* myMotor;
    static MSMeanData* myCars;
    static MSMeanData* myBikes;
    static MSMeanData* myBuses;
    static MSMeanData* myPeds;
    static MSMeanData* myEmissions;
    static MSMeanData* myNoise;
    static SUMOTime myBegin;
    /// @brief congestion cache for coloring
    static std::map<const MSEdge*, double> myCongestionCache;
    static SUMOTime myCongestionCacheTime;
    /// @brief crossings by junction id
    static std::map<std::string, std::vector<const MSEdge*> > myCrossings;
};
