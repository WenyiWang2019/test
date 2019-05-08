/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * <OWNER> = Apple Inc.
 * <ORGANIZATION> = Apple Inc.
 * <YEAR> = 2017
 *
 * Copyright (c) 2017, Apple Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the <ORGANIZATION> nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PCCPointSet_h
#define PCCPointSet_h

#include <assert.h>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include "PCCMath.h"
#include "PCCMisc.h"
#include <thread>

namespace pcc {
#define PI (3.1415926)
#define EPS (1e-15)
#define INF (100000000000000000)

	class PCCPointSet3 {
	 public:
	  PCCPointSet3() {
		withColors = false;
		withReflectances = false;
		isPositionQuantized=false;
	  }
	  PCCPointSet3(bool IsPositionQuantized) {
		  withColors = false;
		  withReflectances = false;
		  isPositionQuantized = IsPositionQuantized;
	  }
	  PCCPointSet3(const PCCPointSet3 &) = default;
	  PCCPointSet3 &operator=(const PCCPointSet3 &rhs) = default;
	  ~PCCPointSet3() = default;
		  
	  PCCPoint3D operator[](const size_t index) const {
		assert(index < positions.size());
		return positions[index];
	  }
	  PCCPoint3D &operator[](const size_t index) {
		assert(index < positions.size());
		return positions[index];
	  }

	  void Unique()
	  {
		  std::vector<PCCPoint3D>::iterator first = positions.begin();
		  std::vector<PCCPoint3D>::iterator last = positions.end();
		  std::vector<PCCColor3B>::iterator first_cor = colors.begin();
		  std::vector<PCCColor3B>::iterator last_cor = colors.end();

		  if (first == last) return ;

		  std::vector<PCCPoint3D>::iterator result = first;
		  std::vector<PCCColor3B>::iterator result_cor = first_cor;
		  while (++first_cor,++first != last)
		  {
			  if (!(*result == *first))
			  {
				  *(++result) = *first;
				  *(++result_cor) = *first_cor;
			  }
		  }

		  positions.erase(++result,last);
		  colors.erase(++result_cor, last_cor);

		  return;
	  }



	  void setPosition(const size_t index, const PCCPoint3D position) {
		assert(index < positions.size());
		positions[index] = position;
	  }
	  PCCColor3B getColor(const size_t index) const {
		assert(index < colors.size() && withColors);
		return colors[index];
	  }
	  PCCColor3B &getColor(const size_t index) {
		assert(index < colors.size() && withColors);
		return colors[index];
	  }
	  void setColor(const size_t index, const PCCColor3B color) {
		assert(index < colors.size() && withColors);
		colors[index] = color;
	  }
	  uint16_t getReflectance(const size_t index) const {
		assert(index < reflectances.size() && withReflectances);
		return reflectances[index];
	  }
	  uint16_t &getReflectance(const size_t index) {
		assert(index < reflectances.size() && withReflectances);
		return reflectances[index];
	  }
	  void setReflectance(const size_t index, const uint16_t reflectance) {
		assert(index < reflectances.size() && withReflectances);
		reflectances[index] = reflectance;
	  }

	  bool hasReflectances() const { return withReflectances; }
	  void addReflectances() {
		withReflectances = true;
		resize(getPointCount());
	  }
	  void removeReflectances() {
		withReflectances = false;
		reflectances.resize(0);
	  }

	  bool hasColors() const { return withColors; }
	  void addColors() {
		withColors = true;
		resize(getPointCount());
	  }
	  void removeColors() {
		withColors = false;
		colors.resize(0);
	  }

	  size_t getPointCount() const { return positions.size(); }
	  void resize(const size_t size) {
		positions.resize(size);
		if (hasColors()) {
		  colors.resize(size);
		}
		if (hasReflectances()) {
		  reflectances.resize(size);
		}
	  }
	  void reserve(const size_t size) {
		positions.reserve(size);
		if (hasColors()) {
		  colors.reserve(size);
		}
		if (hasReflectances()) {
		  reflectances.reserve(size);
		}
	  }
	  void clear() {
		positions.clear();
		colors.clear();
		reflectances.clear();
	  }
	  size_t addPoint(const PCCPoint3D &position) {
		const size_t index = getPointCount();
		resize(index + 1);
		positions[index] = position;
		return index;
	  }
	  void setPoint(const size_t index, const PCCPoint3D &position) {
		  const size_t Pindex = getPointCount();
		  assert(index < Pindex);
		  positions[index] = position;
	  }
	  PCCPoint3D &getPosition(const size_t index) {
		  assert(index < positions.size());
		  return positions[index];
	  }
	  void swapPoints(const size_t index1, const size_t index2) {
		assert(index1 < getPointCount());
		assert(index2 < getPointCount());
		std::swap((*this)[index1], (*this)[index2]);
		if (hasColors()) {
		  std::swap(getColor(index1), getColor(index2));
		}
		if (hasReflectances()) {
		  std::swap(getReflectance(index1), getReflectance(index2));
		}
	  }
	  bool FindPointFromImdex()
	  {
		  return true;
	  }

	  PCCPoint3D computeCentroid() const {
		PCCPoint3D bary(0.0);
		const size_t pointCount = getPointCount();
		if (pointCount) {
		  for (size_t i = 0; i < pointCount; ++i) {
			const PCCPoint3D &pt = (*this)[i];
			bary += pt;
		  }
		  bary /= double(pointCount);
		}
		return bary;
	  }
	  PCCBox3D computeBoundingBox() const {
		PCCBox3D bbox = {std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()};
		const size_t pointCount = getPointCount();
		for (size_t i = 0; i < pointCount; ++i) {
		  const PCCPoint3D &pt = (*this)[i];
		  for (int k = 0; k < 3; ++k) {
			if (pt[k] > bbox.max[k]) {
			  bbox.max[k] = pt[k];
			}
			if (pt[k] < bbox.min[k]) {
			  bbox.min[k] = pt[k];
			}
		  }
		}
		return bbox;
	  }
	  static bool compareSeparators(char aChar, const char *const sep) {
		int i = 0;
		while (sep[i] != '\0') {
		  if (aChar == sep[i]) return false;
		  i++;
		}
		return true;
	  }
	  static inline bool getTokens(const char *str, const char *const sep,
								   std::vector<std::string> &tokens) {
		if (!tokens.empty()) tokens.clear();
		std::string buf = "";
		size_t i = 0;
		size_t length = ::strlen(str);
		while (i < length) {
		  if (compareSeparators(str[i], sep)) {
			buf += str[i];
		  } else if (buf.length() > 0) {
			tokens.push_back(buf);
			buf = "";
		  }
		  i++;
		}
		if (!buf.empty()) tokens.push_back(buf);
		return !tokens.empty();
	  }
	  bool write(const std::string &fileName, const bool asAscii = false) {
		std::ofstream fout(fileName, std::ofstream::out);
		if (!fout.is_open()) {
		  return false;
		}
		const size_t pointCount = getPointCount();
		fout << "ply" << std::endl;

		if (asAscii) {
		  fout << "format ascii 1.0" << std::endl;
		} else {
		  PCCEndianness endianess = PCCSystemEndianness();
		  if (endianess == PCC_BIG_ENDIAN) {
			fout << "format binary_big_endian 1.0" << std::endl;
		  } else {
			fout << "format binary_little_endian 1.0" << std::endl;
		  }
		}
		fout << "element vertex " << pointCount << std::endl;
		fout << "property float64 x" << std::endl;
		fout << "property float64 y" << std::endl;
		fout << "property float64 z" << std::endl;
		if (hasColors()) {
		  fout << "property uchar red" << std::endl;
		  fout << "property uchar green" << std::endl;
		  fout << "property uchar blue" << std::endl;
		}
		if (hasReflectances()) {
		  fout << "property uint16 refc" << std::endl;
		}
		fout << "element face 0" << std::endl;
		fout << "property list uint8 int32 vertex_index" << std::endl;
		fout << "end_header" << std::endl;
		if (asAscii) {
		  fout << std::setprecision(std::numeric_limits<double>::max_digits10);
		  for (size_t i = 0; i < pointCount; ++i) {
			const PCCPoint3D &position = (*this)[i];
			fout << position.x() << " " << position.y() << " " << position.z();
			if (hasColors()) {
			  const PCCColor3B &color = getColor(i);
			  fout << " " << static_cast<int>(color[0]) << " " << static_cast<int>(color[1]) << " "
				   << static_cast<int>(color[2]);
			}
			if (hasReflectances()) {
			  fout << " " << static_cast<int>(getReflectance(i));
			}
			fout << std::endl;
		  }
		} else {
		  fout.clear();
		  fout.close();
		  fout.open(fileName, std::ofstream::binary | std::ofstream::app);
		  for (size_t i = 0; i < pointCount; ++i) {
			const PCCPoint3D &position = (*this)[i];
			fout.write(reinterpret_cast<const char *const>(&position), sizeof(double) * 3);
			if (hasColors()) {
			  const PCCColor3B &color = getColor(i);
			  fout.write(reinterpret_cast<const char *>(&color), sizeof(uint16_t) * 3);
			}
			if (hasReflectances()) {
			  const uint16_t &reflectance = getReflectance(i);
			  fout.write(reinterpret_cast<const char *>(&reflectance), sizeof(uint16_t));
			}
		  }
		}
		fout.close();
		return true;
	  }
	  bool read(const std::string &fileName) {
		std::ifstream ifs(fileName, std::ifstream::in);
		if (!ifs.is_open()) {
		  return false;
		}
		enum AttributeType {
		  ATTRIBUTE_TYPE_FLOAT64 = 0,
		  ATTRIBUTE_TYPE_FLOAT32 = 1,
		  ATTRIBUTE_TYPE_UINT64 = 2,
		  ATTRIBUTE_TYPE_UINT32 = 3,
		  ATTRIBUTE_TYPE_UINT16 = 4,
		  ATTRIBUTE_TYPE_UINT8 = 5,
		  ATTRIBUTE_TYPE_INT64 = 6,
		  ATTRIBUTE_TYPE_INT32 = 7,
		  ATTRIBUTE_TYPE_INT16 = 8,
		  ATTRIBUTE_TYPE_INT8 = 9,
		};
		struct AttributeInfo {
		  std::string name;
		  AttributeType type;
		  size_t byteCount;
		};

		std::vector<AttributeInfo> attributesInfo;
		attributesInfo.reserve(16);
		const size_t MAX_BUFFER_SIZE = 4096;
		char tmp[MAX_BUFFER_SIZE];
		const char *sep = " \t\r";
		std::vector<std::string> tokens;

		ifs.getline(tmp, MAX_BUFFER_SIZE);
		getTokens(tmp, sep, tokens);
		if (tokens.empty() || tokens[0] != "ply") {
		  std::cout << "Error: corrupted file!" << std::endl;
		  return false;
		}
		bool isAscii = false;
		double version = 1.0;
		size_t pointCount = 0;
		bool isVertexProperty = true;
		while (1) {
		  if (ifs.eof()) {
			std::cout << "Error: corrupted header!" << std::endl;
			return false;
		  }
		  ifs.getline(tmp, MAX_BUFFER_SIZE);
		  getTokens(tmp, sep, tokens);
		  if (tokens.empty() || tokens[0] == "comment") {
			continue;
		  }
		  if (tokens[0] == "format") {
			if (tokens.size() != 3) {
			  std::cout << "Error: corrupted format info!" << std::endl;
			  return false;
			}
			isAscii = tokens[1] == "ascii";
			version = atof(tokens[2].c_str());
		  } else if (tokens[0] == "element") {
			if (tokens.size() != 3) {
			  std::cout << "Error: corrupted element info!" << std::endl;
			  return false;
			}
			if (tokens[1] == "vertex") {
			  pointCount = atoi(tokens[2].c_str());
			} else {
			  isVertexProperty = false;
			}
		  } else if (tokens[0] == "property" && isVertexProperty) {
			if (tokens.size() != 3) {
			  std::cout << "Error: corrupted property info!" << std::endl;
			  return false;
			}
			const std::string &propertyType = tokens[1];
			const std::string &propertyName = tokens[2];
			const size_t attributeIndex = attributesInfo.size();
			attributesInfo.resize(attributeIndex + 1);
			AttributeInfo &attributeInfo = attributesInfo[attributeIndex];
			attributeInfo.name = propertyName;
			if (propertyType == "float64") {
			  attributeInfo.type = ATTRIBUTE_TYPE_FLOAT64;
			  attributeInfo.byteCount = 8;
			} else if (propertyType == "float" || propertyType == "float32") {
			  attributeInfo.type = ATTRIBUTE_TYPE_FLOAT32;
			  attributeInfo.byteCount = 4;
			} else if (propertyType == "uint64") {
			  attributeInfo.type = ATTRIBUTE_TYPE_UINT64;
			  attributeInfo.byteCount = 8;
			} else if (propertyType == "uint32") {
			  attributeInfo.type = ATTRIBUTE_TYPE_UINT32;
			  attributeInfo.byteCount = 4;
			} else if (propertyType == "uint16") {
			  attributeInfo.type = ATTRIBUTE_TYPE_UINT16;
			  attributeInfo.byteCount = 2;
			} else if (propertyType == "uchar" || propertyType == "uint8") {
			  attributeInfo.type = ATTRIBUTE_TYPE_UINT8;
			  attributeInfo.byteCount = 1;
			} else if (propertyType == "int64") {
			  attributeInfo.type = ATTRIBUTE_TYPE_INT64;
			  attributeInfo.byteCount = 8;
			} else if (propertyType == "int32") {
			  attributeInfo.type = ATTRIBUTE_TYPE_INT32;
			  attributeInfo.byteCount = 4;
			} else if (propertyType == "int16") {
			  attributeInfo.type = ATTRIBUTE_TYPE_INT16;
			  attributeInfo.byteCount = 2;
			} else if (propertyType == "char" || propertyType == "int8") {
			  attributeInfo.type = ATTRIBUTE_TYPE_INT8;
			  attributeInfo.byteCount = 1;
			}
		  } else if (tokens[0] == "end_header") {
			break;
		  }
		}
		if (version != 1.0) {
		  std::cout << "Error: non-supported version!" << std::endl;
		  return false;
		}

		size_t indexX = PCC_UNDEFINED_INDEX;
		size_t indexY = PCC_UNDEFINED_INDEX;
		size_t indexZ = PCC_UNDEFINED_INDEX;
		size_t indexR = PCC_UNDEFINED_INDEX;
		size_t indexG = PCC_UNDEFINED_INDEX;
		size_t indexB = PCC_UNDEFINED_INDEX;
		size_t indexReflectance = PCC_UNDEFINED_INDEX;
		const size_t attributeCount = attributesInfo.size();
		for (size_t a = 0; a < attributeCount; ++a) {
		  const auto &attributeInfo = attributesInfo[a];
		  if (attributeInfo.name == "x" &&
			  (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4)) {
			indexX = a;
		  } else if (attributeInfo.name == "y" &&
					 (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4)) {
			indexY = a;
		  } else if (attributeInfo.name == "z" &&
					 (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4)) {
			indexZ = a;
		  } else if (attributeInfo.name == "red" && (attributeInfo.byteCount == 1|| attributeInfo.byteCount == 2)) {
			indexR = a;
		  } else if (attributeInfo.name == "green" && (attributeInfo.byteCount == 1 || attributeInfo.byteCount == 2)) {
			indexG = a;
		  } else if (attributeInfo.name == "blue" && (attributeInfo.byteCount == 1 || attributeInfo.byteCount == 2)) {
			indexB = a;
		  } else if ((attributeInfo.name == "reflectance" || attributeInfo.name == "refc") &&
					 attributeInfo.byteCount <= 2) {
			indexReflectance = a;
		  }
		}
		if (indexX == PCC_UNDEFINED_INDEX || indexY == PCC_UNDEFINED_INDEX ||
			indexZ == PCC_UNDEFINED_INDEX) {
		  std::cout << "Error: missing coordinates!" << std::endl;
		  return false;
		}
		withColors = indexR != PCC_UNDEFINED_INDEX && indexG != PCC_UNDEFINED_INDEX &&
					 indexB != PCC_UNDEFINED_INDEX;
		withReflectances = indexReflectance != PCC_UNDEFINED_INDEX;
		resize(pointCount);
		if (isAscii) {
		  size_t pointCounter = 0;
		  while (!ifs.eof() && pointCounter < pointCount) {
			ifs.getline(tmp, MAX_BUFFER_SIZE);
			getTokens(tmp, sep, tokens);
			if (tokens.empty()) {
			  continue;
			}
			if (tokens.size() < attributeCount) {
			  return false;
			}
			auto &position = positions[pointCounter];
			position[0] = atof(tokens[indexX].c_str());
			position[1] = atof(tokens[indexY].c_str());
			position[2] = atof(tokens[indexZ].c_str());
			if (hasColors()) {
			  auto &color = colors[pointCounter];
			  color[0] = atoi(tokens[indexR].c_str());
			  color[1] = atoi(tokens[indexG].c_str());
			  color[2] = atoi(tokens[indexB].c_str());
			}
			if (hasReflectances()) {
			  reflectances[pointCounter] = uint16_t(atoi(tokens[indexReflectance].c_str()));
			}
			++pointCounter;
		  }
		} else {
		  for (size_t pointCounter = 0; pointCounter < pointCount && !ifs.eof(); ++pointCounter) {
			auto &position = positions[pointCounter];
			for (size_t a = 0; a < attributeCount && !ifs.eof(); ++a) {
			  const auto &attributeInfo = attributesInfo[a];
			  if (a == indexX) {
				if (attributeInfo.byteCount == 4) {
				  float x;
				  ifs.read(reinterpret_cast<char *>(&x), sizeof(float));
				  position[0] = x;
				} else {
				  double x;
				  ifs.read(reinterpret_cast<char *>(&x), sizeof(double));
				  position[0] = x;
				}
			  } else if (a == indexY) {
				if (attributeInfo.byteCount == 4) {
				  float y;
				  ifs.read(reinterpret_cast<char *>(&y), sizeof(float));
				  position[1] = y;
				} else {
				  double y;
				  ifs.read(reinterpret_cast<char *>(&y), sizeof(double));
				  position[1] = y;
				}
			  } else if (a == indexZ) {
				if (attributeInfo.byteCount == 4) {
				  float z;
				  ifs.read(reinterpret_cast<char *>(&z), sizeof(float));
				  position[2] = z;
				} else {
				  double z;
				  ifs.read(reinterpret_cast<char *>(&z), sizeof(double));
				  position[2] = z;
				}
			  } else if (a == indexR && attributeInfo.byteCount == 1) {
				auto &color = colors[pointCounter];
				ifs.read(reinterpret_cast<char *>(&color[0]), sizeof(uint8_t));
			  } else if (a == indexG && attributeInfo.byteCount == 1) {
				auto &color = colors[pointCounter];
				ifs.read(reinterpret_cast<char *>(&color[1]), sizeof(uint8_t));
			  } else if (a == indexB && attributeInfo.byteCount == 1) {
				auto &color = colors[pointCounter];
				ifs.read(reinterpret_cast<char *>(&color[2]), sizeof(uint8_t));
			  } else if (a == indexReflectance && attributeInfo.byteCount <= 2) {
				if (indexReflectance == 1) {
				  uint8_t reflectance;
				  ifs.read(reinterpret_cast<char *>(&reflectance), sizeof(uint8_t));
				  reflectances[pointCounter] = reflectance;
				} else {
				  auto &reflectance = reflectances[pointCounter];
				  ifs.read(reinterpret_cast<char *>(reflectance), sizeof(uint16_t));
				}
			  } else {
				char buffer[128];
				ifs.read(buffer, attributeInfo.byteCount);
			  }
			}
		  }
		}
		return true;
	  }
	  void convertRGBToYUV() {  // BT709
		for (auto &color : colors) {
		  const uint16_t r = color[0];
		  const uint16_t g = color[1];
		  const uint16_t b = color[2];
		  const double y = std::round(0.212600 * r + 0.715200 * g + 0.072200 * b);
		  const double u = std::round(-0.114572 * r - 0.385428 * g + 0.500000 * b + 128.0);
		  const double v = std::round(0.500000 * r - 0.454153 * g - 0.045847 * b + 128.0);
		  assert(y >= 0.0 && y <= 255.0 && u >= 0.0 && u <= 255.0 && v >= 0.0 && v <= 255.0);
		  color[0] = static_cast<uint16_t>(y);
		  color[1] = static_cast<uint16_t>(u);
		  color[2] = static_cast<uint16_t>(v);
		}
	  }
	  void convertRGBToYUVClosedLoop() {  // BT709
		for (auto &color : colors) {
		  const uint16_t r = color[0];
		  const uint16_t g = color[1];
		  const uint16_t b = color[2];
		  const double y = std::round(0.212600 * r + 0.715200 * g + 0.072200 * b);
		  const double u = std::round((b - y) / 1.8556 + 128.0);
		  const double v = std::round((r - y) / 1.5748 + 128.0);
		  assert(y >= 0.0 && y <= 255.0 && u >= 0.0 && u <= 255.0 && v >= 0.0 && v <= 255.0);
		  color[0] = static_cast<uint16_t>(y);
		  color[1] = static_cast<uint16_t>(u);
		  color[2] = static_cast<uint16_t>(v);
		}
	  }
	  void convertYUVToRGB() {  // BT709
		for (auto &color : colors) {
		  const double y1 = color[0];
		  const double u1 = color[1] - 128.0;
		  const double v1 = color[2] - 128.0;
		  const double r = PCCClip(round(y1 /*- 0.00000 * u1*/ + 1.57480 * v1), 0.0, 255.0);
		  const double g = PCCClip(round(y1 - 0.18733 * u1 - 0.46813 * v1), 0.0, 255.0);
		  const double b = PCCClip(round(y1 + 1.85563 * u1 /*+ 0.00000 * v1*/), 0.0, 255.0);
		  color[0] = static_cast<uint16_t>(r);
		  color[1] = static_cast<uint16_t>(g);
		  color[2] = static_cast<uint16_t>(b);
		}
	  }
	  
	  void ConvertColorFrom10bTo8b() //将颜色位深从10bit转换为8bit
	  {
		  for (int i = 0; i < colors.size(); i++)
		  {
			  const PCCColor3B &color = getColor(i);
			  PCCColor3B colorTemp;
			  colorTemp[0] = color[0] / 4;
			  colorTemp[1] = color[1] / 4;
			  colorTemp[2] = color[2] / 4;
			  setColor(i, colorTemp);
		  }
	  }
	  void ConvertColorFrom8bTo10b() //将颜色位深从10bit转换为8bit
	  {
		  for (int i = 0; i < colors.size(); i++)
		  {
			  const PCCColor3B &color = getColor(i);
			  PCCColor3B colorTemp;
			  colorTemp[0] = color[0] * 4;
			  colorTemp[1] = color[1] * 4;
			  colorTemp[2] = color[2] * 4;
			  setColor(i, colorTemp);
		  }
	  }

	  void Voxelize(uint8_t PointCloudGeometryBitDepth) //点云体素化
	  {

		  assert(PointCloudGeometryBitDepth <= 16&& !isPositionQuantized);
		  box3D = computeBoundingBox();
		  double xSize = box3D.max.x() - box3D.min.x();
		  double ySize = box3D.max.y() - box3D.min.y();
		  double zSize = box3D.max.z() - box3D.min.z();
		  double maxSize = (xSize > ySize ? xSize : ySize) > zSize ? (xSize > ySize ? xSize : ySize) : zSize;
		  stepSize = maxSize / pow(2, PointCloudGeometryBitDepth);

		  const size_t pointCount = getPointCount();
		  for (int i = 0; i < pointCount; i++)
		  {
			  const PCCPoint3D position = positions[i];
			  PCCPoint3D positionTemp;
			  positionTemp[0] = std::floor((position[0] - box3D.min.x()) / stepSize);
			  positionTemp[1] = std::floor((position[1] - box3D.min.y()) / stepSize);
			  positionTemp[2] = std::floor((position[2] - box3D.min.z()) / stepSize);
			  setPosition(i, positionTemp);
		  }
		  Unique();
		  isPositionQuantized = true;
	  }
	  void Devoxelize()//点云去体素化
	  {
		  assert(isPositionQuantized);
		  const size_t pointCount = getPointCount();
		  for (int i = 0; i < pointCount; i++)
		  {
			  double X = positions[i].x()*stepSize + box3D.min.x();
			  double Y = positions[i].y()*stepSize + box3D.min.y();
			  double Z = positions[i].z()*stepSize + box3D.min.z();
			  PCCPoint3D position(X, Y, Z);
			  positions[i] = position;
		  }
		  isPositionQuantized = false;
	  }
	  size_t addPoints(PCCPointSet3 &pointCloudFrame) //添加点集
	  {
		  const size_t m_pointCount = getPointCount();
		  const size_t pointCount = pointCloudFrame.getPointCount();

		  resize(m_pointCount + pointCount);
		  if (!hasColors()) addColors();
		  for (int i = 0; i < pointCount; i++)
		  {
			  setPosition(m_pointCount + i, pointCloudFrame.getPosition(i));
			  setColor(m_pointCount + i, pointCloudFrame.getColor(i));
		  }
		  return getPointCount();
	  }
	  void ChangeCoordinates(PCCVector3<double> R, PCCVector3<double> T)//变换坐标
	  {
		  const size_t pointCount = getPointCount();
		  for (int i = 0; i < pointCount; i++)
		  {
			  positions[i] = positions[i]+T;
		  }
	  }
	  
	  int sign(double Z)
	  {
		  return (Z > 0 ? 1 : -1);
	  }

	  void ToPolarCoordinates(uint8_t xBitDepth, uint8_t yBitDepth,uint8_t zBitDepth, double Rnear, double Rfar)
	  {
		  //W->X H->Y depth->Z
		  uint16_t W = 1 << xBitDepth;
		  uint16_t H = 1 << yBitDepth;
		  long vmax = (1 << zBitDepth) - 1;

		  double phi, theta;
		  uint16_t n, m, z;
		  const size_t pointCount = getPointCount();
		  for (int i = 0; i < pointCount; i++)
		  {
			  PCCPoint3D &position = getPosition(i);
	
			  double R = sqrt(position.x() * position.x() + position.y() * position.y() + position.z() * position.z());

			  phi = -sign(position.y())*acos(position.x() / sqrt(position.x() * position.x() + position.y() * position.y()));
			  theta = asin(position.z() / R);

			  if (W == H)
			  {
				  n = floor((phi / PI + 0.5)*W - 0.5 + 0.5);
				  n = n >= W ? 0 : n;
				  n = n < 0 ? W - 1 : n;
				  m = floor((0.5 - theta / PI)*H - 0.5 + 0.5);
				  m = m >= H ? 0 : m;
				  m = m < 0 ? H - 1 : m;
			  }
			  else
			  {
				  n = floor((phi / (2 * PI) + 0.5)*W - 0.5 + 0.5);
				  n = n >= W ? 0 : n;
				  n = n < 0 ? W - 1 : n;
				  m = floor((0.5 - theta / PI)*H - 0.5 + 0.5);
				  m = m >= H ? 0 : m;
				  m = m < 0 ? H - 1 : m;
			  }
		
			  z= floor(vmax * (1 / R - 1 / Rfar) / (1 / Rnear - 1 / Rfar) + 0.5);
			  position = PCCPoint3D(n, m, z);
		  }
	  }

	  std::vector<PCCPoint3D> &getPositions() { return positions; }
	  std::vector<PCCColor3B> &getColors() { return colors; }
	  bool &getWithColors() { withColors; }
	  bool &getWithReflectances() { withReflectances; }
	  bool &getIsPositionQuantized() { isPositionQuantized; }
	  PCCBox3D &getBox3D() { return box3D; }
	  double &getStepSize() { return stepSize; }
	  void setVoxelizationParas(PCCBox3D pbox3D, double pstepSize)
	  {
		  isPositionQuantized = true;
		  box3D = pbox3D;
		  stepSize = pstepSize;
	  }

	  void QuickSort(int l, int r) {

		  if (l < r) {
			  int i = l, j = r;
			  PCCPoint3D temp = positions[l];
			  PCCColor3B tempColor = colors[l];
			  while (i < j) {
				  while (i < j&&(temp < positions[j]|| temp == positions[j]))
					  --j;
				  if (i < j)
				  {
					  positions[i++] = positions[j];
					  colors[i++] = colors[j];
				  }
					  

				  while (i<j&&temp>positions[i])
					  ++i;
				  if (i < j)
				  {
					  positions[j--] = positions[i];
					  colors[j--] = colors[i];
				  }
				 
			  }
			  positions[i] = temp;
			  colors[i] = tempColor;

			  QuickSort(l, i - 1);
			  QuickSort(i + 1, r);
		  }
	  }

	  void InsertSort()
	  {
		  std::vector<bool> a(10, true);
		  for (size_t i = 0; i < positions.size(); i++)
		  {
			  if (i / double(positions.size()) >= 0.1&&a[0]) { printf("排序完成10%......\n"); a[0] = false; }
			  if (i / double(positions.size()) >= 0.2&&a[1]) { printf("排序完成20%......\n"); a[1] = false; }
			  if (i / double(positions.size()) >= 0.3&&a[2]) { printf("排序完成30%......\n"); a[2] = false; }
			  if (i / double(positions.size()) >= 0.4&&a[3]) { printf("排序完成40%......\n"); a[3] = false; }
			  if (i / double(positions.size()) >= 0.5&&a[4]) { printf("排序完成50%......\n"); a[4] = false; }
			  if (i / double(positions.size()) >= 0.6&&a[5]) { printf("排序完成60%......\n"); a[5] = false; }
			  if (i / double(positions.size()) >= 0.7&&a[6]) { printf("排序完成70%......\n"); a[6] = false; }
			  if (i / double(positions.size()) >= 0.8&&a[7]) { printf("排序完成80%......\n"); a[7] = false; }
			  if (i / double(positions.size()) >= 0.9&&a[8]) { printf("排序完成90%......\n"); a[8] = false; }
			  if (i / double(positions.size()) >= 0.99&&a[9]) { printf("排序完成100%......\n"); a[9] = false; }

			  size_t min = i;
			  for (size_t j = i + 1; j < positions.size(); j++)
			  {
				  if (positions[j] < positions[min]) min = j;
			  }
			  PCCPoint3D temp = positions[i];
			  positions[i] = positions[min];
			  positions[min] = positions[i];
			  if (withColors)
			  {
				  PCCColor3B tempColor = colors[i];
				  colors[i] = colors[min];
				  colors[min] = tempColor;
			  }
			  if (withReflectances)
			  {
				  uint16_t tempReflectance = reflectances[i];
				  reflectances[i] = reflectances[min];
				  reflectances[min] = tempReflectance;
			  }
		  }
	  }


	  void max_heapify(size_t start, size_t end)
	  {
		  //建立父节点下标和子节点下标
		  size_t dad = start;

		  size_t son = dad * 2 + 1;

		  while (son <= end)
		  {   //若子节点下标在范围内才做比较
			  if (son + 1 <= end && positions[son] < positions[son + 1]) //先比较两个子节点大小，选择最大的
			  {
				  son++;

			  }

			  if (positions[dad] > positions[son]) //如果父节点大于子节点代表调整完毕,直接跳出
			  {
				  return;
			  }
			  else
			  {   //否则交换父子节点的值再继续左右子节点值得比较
				  swapPoints(dad, son);
				  dad = son;
				  son = dad * 2 + 1;
			  }

		  }
	  }

	  void heap_sort()
	  {
		  //初始化，i从最后一个父节点开始调整
		  for (size_t i = positions.size() / 2 - 1; i >= 0; i--)
		  {
			  max_heapify(i, positions.size() - 1);

		  }
		  printf("创建堆完成......\n");

		  std::vector<bool> a(10, true);

		  for (size_t i = positions.size() - 1; i > 0; i--)
		  {
			  swapPoints(0, i);

			  max_heapify(0, i - 1);

			  if (i / double(positions.size()) >= 0.1&&a[0]) { printf("排序完成10%......\n"); a[0] = false; }
			  if (i / double(positions.size()) >= 0.2&&a[1]) { printf("排序完成20%......\n"); a[1] = false; }
			  if (i / double(positions.size()) >= 0.3&&a[2]) { printf("排序完成30%......\n"); a[2] = false; }
			  if (i / double(positions.size()) >= 0.4&&a[3]) { printf("排序完成40%......\n"); a[3] = false; }
			  if (i / double(positions.size()) >= 0.5&&a[4]) { printf("排序完成50%......\n"); a[4] = false; }
			  if (i / double(positions.size()) >= 0.6&&a[5]) { printf("排序完成60%......\n"); a[5] = false; }
			  if (i / double(positions.size()) >= 0.7&&a[6]) { printf("排序完成70%......\n"); a[6] = false; }
			  if (i / double(positions.size()) >= 0.8&&a[7]) { printf("排序完成80%......\n"); a[7] = false; }
			  if (i / double(positions.size()) >= 0.9&&a[8]) { printf("排序完成90%......\n"); a[8] = false; }
			  if (i / double(positions.size()) >= 0.99&&a[9]) { printf("排序完成100%......\n"); a[9] = false; }
		  }
	  }
	  
	  void sort()
	  {
		  heap_sort();
	  }
	  void RemoveRepeatePoints()
	  {
		  size_t i = 0, j = 0, k;
		  for (k = 1; k < positions.size(); k++)
		  {
			  if (positions[k] == positions[j]) continue;
			  PCCPoint3D tempPosition = positions[j];
			  PCCColor3B tempColor(0, 0, 0);
			  for (size_t m = j; m < k; m++)
			  {
				  tempColor += colors[m];
			  }
			  tempColor /= (k - j);
			  positions[i] = tempPosition;
			  colors[i] = tempColor;
			  i++;
			  j = k;
		  }

		  PCCPoint3D tempPosition = positions[j];
		  PCCColor3B tempColor(0, 0, 0);
		  for (size_t m = j; m < k; m++)
		  {
			  tempColor += colors[m];
		  }
		  tempColor /= (k - j);
		  positions[i] = tempPosition;
		  colors[i] = tempColor;
		  i++;
	
		  resize(i);
	  }

	 private:
	  std::vector<PCCPoint3D> positions;
	  std::vector<PCCColor3B> colors;
	  std::vector<uint16_t> reflectances;
	  bool withColors;
	  bool withReflectances;

	  bool isPositionQuantized;
	  PCCBox3D box3D; //点云3D包围盒
	  double stepSize; //步长

	};
}

#endif /* PCCPointSet_h */
