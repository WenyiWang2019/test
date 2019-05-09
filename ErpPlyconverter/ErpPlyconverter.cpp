// ErpPlyconverter.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include<stdio.h>
#include<string>
#include <sstream>
#include <iostream>

#include "PCCPointSet.h"
#include "erp.h"

using namespace pcc;
using namespace erp;


void CameraToWorld(pcc::PCCPoint3D &point3D, pcc::PCCPoint3D T, pcc::PCCPoint3D R);
void ErpToPointCloudPolar(CERPParameter &para);
void ErpToPointCloudPolar(CErpFrame &view, PCCPointSet3 &pointCloud);
void PointCloudToErpPolar(CERPParameter &para);
void PointCloudToErpPolar(PCCPointSet3 &pointCloud, CErpFrame &view);
std::string ToString(uint16_t W, uint16_t H, double Rnear, double Rfar, YUVformat yuvFormat, uint16_t bitDepth, size_t frameCounter);
void ParsePlyName(std::string PLYPath, ErpPara &erpPara, PCCBox3D &box3D, double &stepSize);

int main(int argc, char *argv[]) 
{
	CERPParameter para;
	para.Parse(argc, argv);
	para.PrintParas();
	if (para.mode == 0) //ERP转点云(极坐标)
	{
		ErpToPointCloudPolar(para);
	}
	else if (para.mode == 1) //点云转ERP(极坐标)
	{
		PointCloudToErpPolar(para);
	}
	return 0;
}


void CameraToWorld(pcc::PCCPoint3D &point3D, pcc::PCCPoint3D T, pcc::PCCPoint3D R)
{
	R = -R * PI / 180;

	double x1 = point3D[0] * cos(R[1]) + point3D[2] * (-(sin(R[1])));
	double y1 = point3D[1];
	double z1 = point3D[0] * sin(R[1]) + point3D[2] * cos(R[1]);

	double x2 = x1 * cos(R[0]) + y1 * sin(R[0]);
	double y2 = x1 * (-sin(R[0])) + y1 * cos(R[0]);
	double z2 = z1;

	point3D[0] = x2;
	point3D[1] = y2 * cos(R[2]) + z2 * sin(R[2]);
	point3D[2] = y2 * (-sin(R[2])) + z2 * cos(R[2]);
	point3D = point3D + T;
}

void ErpToPointCloudPolar(CERPParameter &para)
{
	for (size_t i = 0; i < para.frameNumber; i++) //帧数
	{
		printf("正在转换第%d帧......\n", i + 1);

		PCCPointSet3 synPointCloud;

		for (int j = 0; j < para.viewNumber; j++) //每帧视点数
		{
			printf("\t正在合并第%d个视点......\n", j + 1);

			CErpFrame view(para.viewNames[j], para.widths[j], para.heights[j], para.Rnears[j], para.Rfars[j],
				para.depthYUVFormats[j], para.depthbitDepths[j], para.textureYUVFormats[j],
				para.texturebitDepths[j], para.T[j], para.R[j]);

			view.read(para.textureFileNames[j], para.depthFileNames[j], i);
			view.textureYUV420ToYUV444();
			PCCPointSet3 pointCloud;
			ErpToPointCloudPolar(view, pointCloud);
			synPointCloud.addPoints(pointCloud);
		}

		synPointCloud.ChangeCoordinates(para.R[0], -para.T[0]);
		synPointCloud.ToPolarCoordinates(12, 11, 16, para.Rnears[0], para.Rfars[0]);
	
		printf("正在排序......\n");
		synPointCloud.sort();
		printf("排序完成......\n");

		printf("正在去除重复点......\n");
		synPointCloud.RemoveRepeatePoints();
		printf("去除重复点完成......\n");

		synPointCloud.ConvertColorFrom10bTo8b();
		synPointCloud.convertYUVToRGB();

		std::string synPointCloudName = para.outputPlyPath + "\\syn_" +
			ToString(para.widths[0], para.heights[0], para.Rnears[0], para.Rfars[0], para.depthYUVFormats[0],
				para.depthbitDepths[0], i) + ".ply";
		printf("\t正在将点云文件写入磁盘......\n");
		synPointCloud.write(synPointCloudName, true);
	}
}

void ErpToPointCloudPolar(CErpFrame &view, PCCPointSet3 &pointCloud)
{
	auto W = view.getWidth();
	auto H = view.getHeight(); 
	double phi;
	double theta;
	double R;

	long vmax = view.getVmax();
	double Rnear = view.getRnear();
	double Rfar = view.getRfar();

	pcc::PCCPoint3D T = view.getT();
	pcc::PCCPoint3D Rotation = view.getR();

	auto depth = view.getDepth();
	auto texture = view.getTexture();
	pcc::PCCPoint3D point3D;
	pcc::PCCColor3B Color3B;
	uint16_t Depth;
	size_t piontCount = pointCloud.getPointCount();
	pointCloud.resize(piontCount + W * H);

	for (size_t m = 0; m < H; m++)
	{
		for (size_t n = 0; n < W; n++)
		{
			Depth = (*depth)[0][m][n];
			if (W == H)
			{
				phi = ((n + 0.5) / W - 0.5) * PI;
				theta = (0.5 - (m + 0.5) / H)*PI;
			}
			else
			{
				phi = ((n + 0.5) / W - 0.5) * 2 * PI;
				theta = (0.5 - (m + 0.5) / H)*PI;
			}
			if (Depth != 0)
			{
				R = vmax * Rnear*Rfar / (Depth*(Rfar - Rnear) + vmax * Rnear);

				point3D[0] = R * cos(theta)*cos(phi);
				point3D[1] = -R * cos(theta)*sin(phi);
				point3D[2] = R * sin(theta);
			
				//变化坐标系（相机坐标 ---> 世界坐标）
				CameraToWorld(point3D, T, Rotation);

				Color3B[0] = (*texture)[0][m][n];
				Color3B[1] = (*texture)[1][m][n];
				Color3B[2] = (*texture)[2][m][n];

				pointCloud.setPoint(piontCount, point3D);
				pointCloud.setColor(piontCount, Color3B);
				piontCount++;
			}
		}
	}
	pointCloud.resize(piontCount);
}

void PointCloudToErpPolar(PCCPointSet3 &pointCloud, CErpFrame &view)
{
	
}

void PointCloudToErpPolar(CERPParameter &para)
{

	
}
std::string ToString(uint16_t W, uint16_t H, double Rnear, double Rfar, YUVformat yuvFormat,uint16_t bitDepth,size_t frameCounter)
{
	std::stringstream path;
	std::stringstream tempstrRnear;
	std::stringstream tempstrRfar;

	tempstrRnear << std::fixed << std::setprecision(1) << Rnear;
	tempstrRfar << std::fixed << std::setprecision(1) << Rfar;


	std::vector<std::string> tempstrRnearSplit = splitWithStl(tempstrRnear.str(), ".");
	std::vector<std::string> tempstrRfarSplit = splitWithStl(tempstrRfar.str(), ".");


	path << W << "_" << H << "_" << tempstrRnearSplit[0] << "_";
	path << tempstrRnearSplit[1] << "_" << tempstrRfarSplit[0] << "_" << tempstrRfarSplit[1] << "_";
	if (yuvFormat == YUV420) path << "420_" << bitDepth << "b_f" << frameCounter;
	else if (yuvFormat == YUV444) path << "444_" << bitDepth << "b_f" << frameCounter;
	else exit(1);
	return path.str();
}

void ParsePlyName(std::string PLYPath, ErpPara &erpPara, PCCBox3D &box3D, double &stepSize)
{
	std::vector<std::string> PLYPathSplit = splitWithStl(PLYPath, "\\");
	PLYPathSplit = splitWithStl(PLYPathSplit.back(), "_");
	assert(PLYPathSplit.size() == 18);
	erpPara.W = atoi(PLYPathSplit[1].c_str());
	erpPara.H = atoi(PLYPathSplit[2].c_str());
	erpPara.Rnear = atof((PLYPathSplit[3] + "." + PLYPathSplit[4]).c_str());
	erpPara.Rfar = atof((PLYPathSplit[5] + "." + PLYPathSplit[6]).c_str());
	stepSize = atof((PLYPathSplit[9] + "." + PLYPathSplit[10]).c_str());
	double X = atof((PLYPathSplit[11] + "." + PLYPathSplit[12]).c_str());
	double Y = atof((PLYPathSplit[13] + "." + PLYPathSplit[14]).c_str());
	double Z = atof((PLYPathSplit[15] + "." + PLYPathSplit[16]).c_str());
	box3D.min.SetXYZ(X, Y, Z);
	if (PLYPathSplit[7] == "420") erpPara.textureYUVFormat = YUV420, erpPara.depthYUVFormat = YUV420;
	else if(PLYPathSplit[7] == "444") erpPara.textureYUVFormat = YUV444, erpPara.depthYUVFormat = YUV444;
	else exit(1);
	 
	std::vector<std::string> PLYPathSplit16Split = splitWithStl(PLYPathSplit[8], "b");
	assert(PLYPathSplit16Split.size() == 2);
	erpPara.texturebitDepth = atoi(PLYPathSplit16Split[0].c_str()), erpPara.depthbitDepth = atoi(PLYPathSplit16Split[0].c_str());
}

