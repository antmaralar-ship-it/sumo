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
        std::string los;        // level of service A-F
    };

    /// @brief register the options
    static void insertOptions(OptionsCont& oc);

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

private:
    static void compute(const MSEdge* edge, Key& key, std::vector<std::string>* out);
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
    /// @brief crossings by junction id
    static std::map<std::string, std::vector<const MSEdge*> > myCrossings;
};
