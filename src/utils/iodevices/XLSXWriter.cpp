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
/// @file    XLSXWriter.cpp
/// @date    November 2026
///
// Minimal writer for Excel workbooks (.xlsx) without external libraries
/****************************************************************************/
#include <config.h>

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <utils/xml/XMLSubSys.h>
#include "XLSXWriter.h"


// ===========================================================================
// helpers
// ===========================================================================
namespace {

uint32_t crc32(const std::string& data) {
    static uint32_t table[256];
    static bool init = false;
    if (!init) {
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for (int k = 0; k < 8; k++) {
                c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            }
            table[i] = c;
        }
        init = true;
    }
    uint32_t c = 0xFFFFFFFFu;
    for (unsigned char b : data) {
        c = table[(c ^ b) & 0xFF] ^ (c >> 8);
    }
    return c ^ 0xFFFFFFFFu;
}

void put16(std::string& s, uint16_t v) {
    s.push_back((char)(v & 0xFF));
    s.push_back((char)((v >> 8) & 0xFF));
}

void put32(std::string& s, uint32_t v) {
    for (int i = 0; i < 4; i++) {
        s.push_back((char)((v >> (8 * i)) & 0xFF));
    }
}

/// @brief builds a zip archive with stored (uncompressed) entries
std::string zip(const std::vector<std::pair<std::string, std::string> >& files) {
    std::string out, central;
    const uint16_t time = 0, date = (uint16_t)(((2026 - 1980) << 9) | (1 << 5) | 1);
    for (const auto& f : files) {
        const uint32_t crc = crc32(f.second);
        const uint32_t offset = (uint32_t)out.size();
        put32(out, 0x04034b50);
        put16(out, 20);
        put16(out, 0x0800); // UTF-8 names
        put16(out, 0);      // stored
        put16(out, time);
        put16(out, date);
        put32(out, crc);
        put32(out, (uint32_t)f.second.size());
        put32(out, (uint32_t)f.second.size());
        put16(out, (uint16_t)f.first.size());
        put16(out, 0);
        out += f.first;
        out += f.second;
        put32(central, 0x02014b50);
        put16(central, 20);
        put16(central, 20);
        put16(central, 0x0800);
        put16(central, 0);
        put16(central, time);
        put16(central, date);
        put32(central, crc);
        put32(central, (uint32_t)f.second.size());
        put32(central, (uint32_t)f.second.size());
        put16(central, (uint16_t)f.first.size());
        put16(central, 0);
        put16(central, 0);
        put16(central, 0);
        put16(central, 0);
        put32(central, 0);
        put32(central, offset);
        central += f.first;
    }
    const uint32_t cdOffset = (uint32_t)out.size();
    out += central;
    put32(out, 0x06054b50);
    put16(out, 0);
    put16(out, 0);
    put16(out, (uint16_t)files.size());
    put16(out, (uint16_t)files.size());
    put32(out, (uint32_t)central.size());
    put32(out, cdOffset);
    put16(out, 0);
    return out;
}

} // namespace


// ===========================================================================
// method definitions
// ===========================================================================
void
XLSXWriter::addSheet(const std::string& name, const std::vector<Row>& rows, const std::vector<double>& widths) {
    // Excel limits sheet names to 31 characters without []:*?/\ .
    std::string n;
    for (char c : name) {
        if (std::string("[]:*?/\\").find(c) == std::string::npos) {
            n += c;
        }
    }
    if (n.size() > 31) {
        n = n.substr(0, 31);
    }
    mySheets.push_back({n.empty() ? "Hoja" + std::to_string(mySheets.size() + 1) : n, rows, widths});
}


std::string
XLSXWriter::escape(const std::string& s) {
    std::string r;
    for (char c : s) {
        switch (c) {
            case '&':
                r += "&amp;";
                break;
            case '<':
                r += "&lt;";
                break;
            case '>':
                r += "&gt;";
                break;
            case '"':
                r += "&quot;";
                break;
            default:
                if ((unsigned char)c >= 0x20 || c == '\n' || c == '\t') {
                    r += c;
                }
        }
    }
    return r;
}


std::string
XLSXWriter::column(int index) {
    std::string r;
    index++;
    while (index > 0) {
        const int m = (index - 1) % 26;
        r.insert(r.begin(), (char)('A' + m));
        index = (index - 1) / 26;
    }
    return r;
}


std::string
XLSXWriter::sheetXML(const Sheet& sheet) const {
    std::ostringstream x;
    x.precision(15);
    x << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n"
      << "<worksheet xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\">";
    if (!sheet.widths.empty()) {
        x << "<cols>";
        for (int i = 0; i < (int)sheet.widths.size(); i++) {
            x << "<col min=\"" << i + 1 << "\" max=\"" << i + 1 << "\" width=\"" << sheet.widths[i] << "\" customWidth=\"1\"/>";
        }
        x << "</cols>";
    }
    x << "<sheetData>";
    for (int r = 0; r < (int)sheet.rows.size(); r++) {
        x << "<row r=\"" << r + 1 << "\">";
        for (int c = 0; c < (int)sheet.rows[r].size(); c++) {
            const Cell& cell = sheet.rows[r][c];
            const std::string ref = column(c) + std::to_string(r + 1);
            if (cell.isNumber) {
                x << "<c r=\"" << ref << "\" s=\"" << (int)cell.style << "\"><v>" << cell.number << "</v></c>";
            } else if (!cell.text.empty() || cell.style != NORMAL) {
                x << "<c r=\"" << ref << "\" s=\"" << (int)cell.style << "\" t=\"inlineStr\"><is><t xml:space=\"preserve\">"
                  << escape(cell.text) << "</t></is></c>";
            }
        }
        x << "</row>";
    }
    x << "</sheetData></worksheet>";
    return x.str();
}


bool
XLSXWriter::save(const std::string& file, std::string& error) const {
    const std::string types =
        "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n"
        "<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">"
        "<Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>"
        "<Default Extension=\"xml\" ContentType=\"application/xml\"/>"
        "<Override PartName=\"/xl/workbook.xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml\"/>"
        "<Override PartName=\"/xl/styles.xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml\"/>";
    std::string sheetsTypes, workbookSheets, workbookRels;
    std::vector<std::pair<std::string, std::string> > files;
    for (int i = 0; i < (int)mySheets.size(); i++) {
        const std::string n = std::to_string(i + 1);
        sheetsTypes += "<Override PartName=\"/xl/worksheets/sheet" + n + ".xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml\"/>";
        workbookSheets += "<sheet name=\"" + escape(mySheets[i].name) + "\" sheetId=\"" + n + "\" r:id=\"rId" + n + "\"/>";
        workbookRels += "<Relationship Id=\"rId" + n + "\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet\" Target=\"worksheets/sheet" + n + ".xml\"/>";
    }
    const std::string stylesId = "rId" + std::to_string(mySheets.size() + 1);
    files.push_back({"[Content_Types].xml", types + sheetsTypes + "</Types>"});
    files.push_back({"_rels/.rels",
                     "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n"
                     "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">"
                     "<Relationship Id=\"rId1\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument\" Target=\"xl/workbook.xml\"/>"
                     "</Relationships>"});
    files.push_back({"xl/workbook.xml",
                     "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n"
                     "<workbook xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\" "
                     "xmlns:r=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships\"><sheets>"
                     + workbookSheets + "</sheets></workbook>"});
    files.push_back({"xl/_rels/workbook.xml.rels",
                     "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n"
                     "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">"
                     + workbookRels +
                     "<Relationship Id=\"" + stylesId + "\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles\" Target=\"styles.xml\"/>"
                     "</Relationships>"});
    // styles: 0 normal, 1 header, 2 good, 3 fair, 4 bad, 5 wrap, 6 number, 7 title
    files.push_back({"xl/styles.xml",
                     "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n"
                     "<styleSheet xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\">"
                     "<numFmts count=\"1\"><numFmt numFmtId=\"164\" formatCode=\"#,##0.00\"/></numFmts>"
                     "<fonts count=\"3\"><font><sz val=\"11\"/><name val=\"Calibri\"/></font>"
                     "<font><b/><sz val=\"11\"/><name val=\"Calibri\"/></font>"
                     "<font><b/><sz val=\"14\"/><name val=\"Calibri\"/></font></fonts>"
                     "<fills count=\"6\"><fill><patternFill patternType=\"none\"/></fill><fill><patternFill patternType=\"gray125\"/></fill>"
                     "<fill><patternFill patternType=\"solid\"><fgColor rgb=\"FFD9E1EA\"/></patternFill></fill>"
                     "<fill><patternFill patternType=\"solid\"><fgColor rgb=\"FFC6EFCE\"/></patternFill></fill>"
                     "<fill><patternFill patternType=\"solid\"><fgColor rgb=\"FFFFEB9C\"/></patternFill></fill>"
                     "<fill><patternFill patternType=\"solid\"><fgColor rgb=\"FFFFC7CE\"/></patternFill></fill></fills>"
                     "<borders count=\"2\"><border/><border><left style=\"thin\"><color rgb=\"FFBFBFBF\"/></left><right style=\"thin\"><color rgb=\"FFBFBFBF\"/></right>"
                     "<top style=\"thin\"><color rgb=\"FFBFBFBF\"/></top><bottom style=\"thin\"><color rgb=\"FFBFBFBF\"/></bottom></border></borders>"
                     "<cellStyleXfs count=\"1\"><xf numFmtId=\"0\" fontId=\"0\" fillId=\"0\" borderId=\"0\"/></cellStyleXfs>"
                     "<cellXfs count=\"8\">"
                     "<xf fontId=\"0\" fillId=\"0\" borderId=\"1\" applyBorder=\"1\" applyAlignment=\"1\"><alignment vertical=\"top\" wrapText=\"1\"/></xf>"
                     "<xf fontId=\"1\" fillId=\"2\" borderId=\"1\" applyFont=\"1\" applyFill=\"1\" applyBorder=\"1\" applyAlignment=\"1\"><alignment vertical=\"top\" wrapText=\"1\"/></xf>"
                     "<xf fontId=\"1\" fillId=\"3\" borderId=\"1\" applyFont=\"1\" applyFill=\"1\" applyBorder=\"1\" applyAlignment=\"1\"><alignment vertical=\"top\" wrapText=\"1\"/></xf>"
                     "<xf fontId=\"1\" fillId=\"4\" borderId=\"1\" applyFont=\"1\" applyFill=\"1\" applyBorder=\"1\" applyAlignment=\"1\"><alignment vertical=\"top\" wrapText=\"1\"/></xf>"
                     "<xf fontId=\"1\" fillId=\"5\" borderId=\"1\" applyFont=\"1\" applyFill=\"1\" applyBorder=\"1\" applyAlignment=\"1\"><alignment vertical=\"top\" wrapText=\"1\"/></xf>"
                     "<xf fontId=\"0\" fillId=\"0\" borderId=\"1\" applyBorder=\"1\" applyAlignment=\"1\"><alignment vertical=\"top\" wrapText=\"1\"/></xf>"
                     "<xf numFmtId=\"164\" fontId=\"0\" fillId=\"0\" borderId=\"1\" applyNumberFormat=\"1\" applyBorder=\"1\" applyAlignment=\"1\"><alignment vertical=\"top\"/></xf>"
                     "<xf fontId=\"2\" fillId=\"0\" borderId=\"0\" applyFont=\"1\"/>"
                     "</cellXfs><cellStyles count=\"1\"><cellStyle name=\"Normal\" xfId=\"0\" builtinId=\"0\"/></cellStyles></styleSheet>"});
    for (int i = 0; i < (int)mySheets.size(); i++) {
        files.push_back({"xl/worksheets/sheet" + std::to_string(i + 1) + ".xml", sheetXML(mySheets[i])});
    }
    const std::string data = zip(files);
    std::ofstream out(XMLSubSys::transcodeToLocal(file).c_str(), std::ios::binary);
    if (!out.good()) {
        error = "No se puede escribir el archivo '" + file + "'. Comprueba que no est\303\241 abierto en Excel.";
        return false;
    }
    out.write(data.data(), (std::streamsize)data.size());
    out.close();
    if (!out.good()) {
        error = "Error al escribir el archivo '" + file + "'.";
        return false;
    }
    return true;
}


/****************************************************************************/
