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
/// @file    XLSXWriter.h
/// @date    November 2026
///
// Minimal writer for Excel workbooks (.xlsx) without external libraries
/****************************************************************************/
#pragma once
#include <config.h>

#include <string>
#include <vector>


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class XLSXWriter
 * @brief Writes simple Excel workbooks (text and number cells, a few styles)
 *
 * The workbook is stored as an uncompressed zip archive, which Excel and
 * LibreOffice open without problems.
 */
class XLSXWriter {
public:
    /// @brief cell styles
    enum Style {
        NORMAL = 0,
        HEADER = 1,   // bold on grey
        GOOD = 2,     // green background
        FAIR = 3,     // yellow background
        BAD = 4,      // red background
        WRAP = 5,     // wrapped text
        NUMBER = 6,   // number with two decimals
        TITLE = 7     // bold, larger
    };

    struct Cell {
        Cell(const std::string& t, Style s = NORMAL) : text(t), style(s) {}
        Cell(double v, Style s = NUMBER) : isNumber(true), number(v), style(s) {}
        std::string text;
        bool isNumber = false;
        double number = 0;
        Style style = NORMAL;
    };

    typedef std::vector<Cell> Row;

    /// @brief adds a worksheet; widths are column widths in characters
    void addSheet(const std::string& name, const std::vector<Row>& rows, const std::vector<double>& widths);

    /// @brief writes the workbook; returns false and an error message on failure
    bool save(const std::string& file, std::string& error) const;

private:
    struct Sheet {
        std::string name;
        std::vector<Row> rows;
        std::vector<double> widths;
    };
    std::vector<Sheet> mySheets;

    static std::string escape(const std::string& s);
    static std::string column(int index);
    std::string sheetXML(const Sheet& sheet) const;
};
